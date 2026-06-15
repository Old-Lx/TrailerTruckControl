import math
from arx_truck_ai import bng_open
from beamngpy import Vehicle
import numpy as np
import scipy.linalg as la
import time
from arx_truck_ai.debugging_sys import registrar_estado, generar_grafico_evaluacion

def calcular_matrices_lqr_obs(V0, D0, x3_delta_F2):
    """
    Calcula dinámicamente o senta las bases matemáticas para la 
    obtención de las matrices K (Ganancia LQR) y L (Observador Luenberger).
    Implementa computacionalmente en Python el modelo de "modelaje.m",
    capaz de ser invocado localmente con el array vectorial del camión.
    """
    # Evitamos singularidades si el camión se detiene por completo
    v_eff = V0 if abs(V0) > 0.1 else 0.1
    
    # Modelo Nominal Continuo (Marcha en Retroceso)
    A_bwd = np.array([
        [0,      -v_eff,          0        ],
        [0,       0,          -v_eff/D0    ],
        [0,   v_eff/D0,       -v_eff/D0   ]  # A[2,1] = +v/D0, A[2,2] = -v/D0
    ])

    B_bwd = np.array([
        [0],
        [0],
        [-v_eff]
    ])

    C = np.array([[1, 0, 0]])
    
    # Aumentamos el modelo añadiendo un estado extra para el error integral
    A_aug = np.zeros((4, 4))
    A_aug[0:3, 0:3] = A_bwd

    # Integrador
    A_aug[3, 0:3] = -C

    B_aug = np.zeros((4, 1))
    B_aug[0:3, :] = B_bwd
    
    # Matrices de penalización extraídas de modelaje.m
    # Penalización
    Q_lqr = np.diag([
        5000,   # error lateral
        100,   # error angular
        8000,    # articulación
        100     # integrador
    ])

    if (abs(x3_delta_F2) < 0.03):       
        R_lqr = np.array([[5000]])
    else:
        R_lqr = np.array([[500]])
    
    # Cálculo de Ganancias Óptimas LQR usando Ecuación de Riccati Continua (CARE)
    X_lqr = la.solve_continuous_are(
        A_aug,
        B_aug,
        Q_lqr,
        R_lqr
    )
    K_aug = np.linalg.inv(R_lqr) @ B_aug.T @ X_lqr
    
    # Diseño del Observador de Estados (Filtro de Kalman / LQR Dual)
    # Asumimos que sólo medimos y2 (la cámara estima). Penalizamos error de estimación.
    Q_obs = 10.0 * np.eye(3)
    R_obs = np.array([[1]])
    X_obs = la.solve_continuous_are(A_bwd.T, C.T, Q_obs, R_obs)
    L_obs = (np.linalg.inv(R_obs) @ C @ X_obs).T
    
    return K_aug, L_obs

def control_lqr_reversa(pos_trailer, psi_trailer, delta_F2, delta_F2_rate, trailer_roll, trailer_roll_rate, 
                        velocidad, ruta, current_idx, dt, integrador_error, x_hat, previous_steering,
                        min_lookahead=5.0, max_lookahead=15.0, k_lookahead_gain=3.5, D0=5.0):
    """
    Control LQR Óptimo con variables de estado y Observador (Luenberger básico).
    Retorna (steering, throttle, brake, next_idx, finished, nuevo_integrador, x_hat_nuevo).
    
    Estados del controlador MIMO:
    x1: Error lateral (desviación espacial hacia la ruta)
    x2: Error angular (heading error del remolque)
    x3: Ángulo de articulación (delta_F2)
    x4: Integrador del error (chattering prevention)
    """
    if len(ruta) == 0:
        return 0.0, 0.0, 1.0, current_idx, True, integrador_error, x_hat

    # 1. Encontrar el punto más cercano para el TRÁILER
    min_dist = float('inf')
    closest_idx = current_idx
    search_limit = min(current_idx + 100, len(ruta))
    for i in range(current_idx, search_limit):
        pt = ruta[i]
        dist = math.hypot(pos_trailer[0] - pt[0], pos_trailer[1] - pt[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    # Si estamos en el tramo final
    if closest_idx >= len(ruta) - 2 and min_dist < 2.0:
        return 0.0, 0.0, 1.0, closest_idx, True, integrador_error, x_hat
    
    # B. Error lateral (x1 o "cross track error") 
    # Proyección ortogonal directa de la distancia entre el tráiler y el segmento de la ruta para evitar zonas muertas angulares
    pt_actual_tr = ruta[closest_idx]
    pt_siguiente_tr = ruta[min(closest_idx + 1, len(ruta) - 1)]
    dx_path = pt_siguiente_tr[0] - pt_actual_tr[0]
    dy_path = pt_siguiente_tr[1] - pt_actual_tr[1]
    yaw_path = math.atan2(dy_path, dx_path)
    dx_tr = pos_trailer[0] - pt_actual_tr[0]
    dy_tr = pos_trailer[1] - pt_actual_tr[1]
    x1_error_lateral = dy_tr * math.cos(yaw_path) - dx_tr * math.sin(yaw_path)

    # 2. Lookahead: solo se usa para avanzar el índice, no para calcular x2
    dist_error = abs(x1_error_lateral)
    lookahead = np.clip(min_lookahead + (k_lookahead_gain * abs(velocidad)) - (dist_error * 2), 2.0, 10.0)
    # (El bloque de búsqueda de target_idx puede eliminarse si no se usa target_pt en ningún otro lado)

    # 3. Errores para el modelo LQR

    # A. Error angular (x2): alinear el tráiler con la dirección del segmento de ruta
    psi_trailer_reversa = psi_trailer + math.pi  # el tráiler se mueve hacia atrás
    x2_error_angular = math.atan2(
        math.sin(yaw_path - psi_trailer_reversa),
        math.cos(yaw_path - psi_trailer_reversa)
    )

    # C. Articulación
    x3_delta_F2 = np.clip(delta_F2, -1.1, 1.1)

    # D. Acumulador integral (x4)
    if abs(x1_error_lateral) < 0.03: 
        integrador_error += x1_error_lateral * dt
    # Limitamos la cuerda del integrador para evitar Wind-up catastrófico
    integrador_error = np.clip(integrador_error, -0.15, 0.15)

    velocidad_abs = abs(velocidad)

    # Vector de Estados Medidos (Físicos Directos o Aproximados por Cámara para el observador)
    y_medido = np.array([[x1_error_lateral]]) # El observador (L_obs) del script de MATLAB mide C = [1,0,0], asume que medimos solo error lateral cross-track.

    # Cálculo dinámico basado en la matriz de Riccati (Computado On-The-Fly desde base de MATLAB modelaje.m)
    # D0 viene del parámetro de la función — cambiar D0_NOMINAL en main() para calibrar
    K_mimo, L_obs = calcular_matrices_lqr_obs(velocidad_abs, D0, x3_delta_F2)

    # Inyección directa de las lecturas físicas de los sensores de orientación y articulación
    # Evita que el sistema asuma estados lineales nulos cuando la dinámica entra en el régimen no lineal del efecto tijera

    # Zona muerta suave sobre x2: cuando x3 está cerca de cero (recuperándose),
    # reducir el peso de x2 para que no domine y arrastre al volante en dirección
    # incorrecta justo al cruzar el cero. K[1]*x2 se silencia progresivamente
    # debajo de 0.15 rad de articulación.
    peso_x2 = float(np.clip(abs(x3_delta_F2) / 0.15, 0.0, 1.0))
    x2_efectivo = x2_error_angular * peso_x2

    x_hat[0,0] = x1_error_lateral
    x_hat[1,0] = x2_efectivo        # x2 pesado según la magnitud de x3
    x_hat[2,0] = x3_delta_F2
    x_hat[3,0] = integrador_error
    
    # 5. Ganancias MIMO y Ejecución LQR
    # Cálculo formal óptimo: u = -K * x 
    u_lqr = - float(K_mimo @ x_hat) # Hay que adecuar el valor dentro de los límites

    # Amortiguamiento activo de la articulación — frena el crecimiento de x3
    max_ruedas_rad = 0.6
    steering_raw = -(u_lqr / max_ruedas_rad)
    
    # PREVENCIÓN EXTRA: Sistema de Suspensión Dinámica (8-DOF Active Control Anti-Roll)
    limite_roll = 0.08
    limite_roll_rate = 0.15
    atenuacion_dinamica = 1.0
    if abs(trailer_roll) > limite_roll or abs(trailer_roll_rate) > limite_roll_rate:
        exceso = max(abs(trailer_roll) / limite_roll, abs(trailer_roll_rate) / limite_roll_rate)
        atenuacion_dinamica = np.clip(1.0 / exceso, 0.4, 1.0)

    # Castigamos u también en el modelo LQR porque la dinámica "Soft Body" no está tabulada en A y B.
    steering_raw *= atenuacion_dinamica

    # if abs(delta_F2) > 0.2: # esta fue una medida desesperada, pero no debería usarse porque solamente evita el jackknife
    #    steering_raw = -np.sign(delta_F2) * 0.9 # Contra-volante forzado para evitar jackknife

    steering = np.clip(steering_raw, -1.0, 1.0)
    
    # Bucle de velocidad básico
    velocidad_objetivo = 2.0 * atenuacion_dinamica
    error_vel = velocidad_objetivo - velocidad
    kp_vel = 0.4
    if error_vel > 0:
        throttle = np.clip(0.1 + kp_vel * error_vel, 0.0, 1.0)
        brake = 0.0
    else:
        throttle = 0.0
        brake = np.clip(-kp_vel * error_vel * 0.5, 0.0, 1.0)

    return steering, throttle, brake, closest_idx, False, integrador_error, x_hat, x1_error_lateral, x2_error_angular, x3_delta_F2, steering_raw


def main():
    truck_trailer, orig = bng_open.main()

    truck_trailer.bng.control.pause()
    truck_trailer.bng.settings.set_deterministic(60)

    # Al poner la caja de cambios del camión en Reversa (Gear = -1), el acelerador lo desplazará hacia atrás
    truck_trailer.truck.control(gear=-1)
    truck_trailer.bng.control.resume()

    current_idx = 0
    route_points = truck_trailer.route

    # Inicialización de estado para LQR (Observador, Integrador y Derivadas)
    last_time = time.time()
    previous_delta_F2 = 0.0
    previous_trailer_roll = 0.0
    previous_steering = 0.0
    
    # Estructura del Estado Estimado (Error lateral, Error angular, Articulación, Error Integral)
    x_hat = np.zeros((4, 1))
    integrador_error = 0.0

    delta_F2_rate_filtrada = 0.0
    alpha_rate = 0.3  # 0=sin filtro, 1=completamente suavizado

    # ── Diagnóstico de D0 ──────────────────────────────────────────────────────
    # En la primera lectura de sensores, imprimir delta_F2 inicial y velocidad
    # para confirmar que el sensor de articulación funciona correctamente.
    # Si tienes acceso a pos_hitch o pos_cabina, calcular D0_real aquí.
    # Probar D0 = 3.0, 4.0, 5.0, 6.0, 7.0 cambiando el valor en la llamada
    # a calcular_matrices_lqr_obs(velocidad_abs, D0_NOMINAL, x3_delta_F2).
    D0_NOMINAL = 3.0  # ← cambiar este valor entre pruebas para calibrar D0
    # ──────────────────────────────────────────────────────────────────────────

    primer_tick = True

    start_time_log = time.time()

    while(True):
        sensores = truck_trailer.read_sensors(is_reverse=True)
        current_time = time.time()
        dt = current_time - last_time
        if dt < 1e-4: dt = 1e-4 

        velocidad = sensores["v1"]
        psi_trailer = sensores["psi_trailer"]
        delta_F2 = sensores["delta_F2"]
        pos_trailer = sensores["trailer_pos"]
        trailer_roll = sensores["trailer_roll"]

        if primer_tick:
            print(f"[D0-DIAG] D0_NOMINAL={D0_NOMINAL} | delta_F2_inicial={delta_F2:.4f} rad | velocidad={velocidad:.3f} m/s")
            print(f"[D0-DIAG] pos_trailer={pos_trailer}")
            primer_tick = False

        # Tasas de cambio (no se integran directo en LQR A/B, pero se guardan por seguridad de atenuación dinámica de masas y roll mode)
        delta_F2_rate_cruda = (delta_F2 - previous_delta_F2) / dt
        delta_F2_rate_filtrada = alpha_rate * delta_F2_rate_filtrada + (1 - alpha_rate) * delta_F2_rate_cruda
        trailer_roll_rate = (trailer_roll - previous_trailer_roll) / dt
        
        previous_delta_F2 = delta_F2
        previous_trailer_roll = trailer_roll
        last_time = current_time

        # Control Estado LQR (MIMO + Observador)
        steering, throttle, brake, current_idx, finished, integrador_error, x_hat, x1, x2, x3, s_raw = control_lqr_reversa(
        pos_trailer, psi_trailer, delta_F2, delta_F2_rate_filtrada, trailer_roll, trailer_roll_rate, 
        velocidad, route_points, current_idx, dt, integrador_error, x_hat, previous_steering=previous_steering,
        min_lookahead=2.0, max_lookahead=8.0, k_lookahead_gain=1.0, D0=D0_NOMINAL
    )

        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)
        
        # Calcular tiempo relativo y guardar estado
        t_sim = current_time - start_time_log
        registrar_estado(t_sim, x1, x2, x3, s_raw)

        previous_steering = steering

        if finished:
            print("Ruta de reversa LQR completada. Frenando...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            break
        elif abs(delta_F2) > 1.0: # Condición de Jackknife (aprox 57 grados)
            print(f"¡Jackknife inminente detectado! (Articulación: {delta_F2:.2f} rad). Abortando para graficar...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            break

    # Cuando salimos del bucle, graficamos
    generar_grafico_evaluacion()
    input('Presione enter cuando termine la simulación en reversa (MIMO-LQR)...')

if __name__ == "__main__":
    main()