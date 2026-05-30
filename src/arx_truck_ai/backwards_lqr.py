import math
from arx_truck_ai import bng_open
from beamngpy import Vehicle
import numpy as np
import scipy.linalg as la
import time

def calcular_matrices_lqr_obs(V0, D0):
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
        [0, -v_eff, 0],
        [0,      0, -v_eff/D0],
        [0,      0,  v_eff/D0]
    ])
    B_bwd = np.array([[0], [0], [-v_eff]])
    C = np.array([[1, 0, 0]])
    
    # Aumentamos el modelo añadiendo un estado extra para el error integral
    A_aug = np.zeros((4, 4))
    A_aug[0:3, 0:3] = A_bwd
    A_aug[3, 0:3] = -C
    B_aug = np.zeros((4, 1))
    B_aug[0:3, :] = B_bwd
    
    # Matrices de penalización extraídas de modelaje.m
    Q_lqr = np.diag([10, 100, 1000, 500])
    R_lqr = np.array([[1]])
    
    # Cálculo de Ganancias Óptimas LQR usando Ecuación de Riccati Continua (CARE)
    X_lqr = la.solve_continuous_are(A_aug, B_aug, Q_lqr, R_lqr)
    K_aug = np.linalg.inv(R_lqr) @ B_aug.T @ X_lqr
    
    # Diseño del Observador de Estados (Filtro de Kalman / LQR Dual)
    # Asumimos que sólo medimos y2 (la cámara estima). Penalizamos error de estimación.
    Q_obs = 1e6 * np.eye(3)
    R_obs = np.array([[1]])
    X_obs = la.solve_continuous_are(A_bwd.T, C.T, Q_obs, R_obs)
    L_obs = (np.linalg.inv(R_obs) @ C @ X_obs).T
    
    return K_aug, L_obs

def control_lqr_reversa(pos_trailer, psi_trailer, delta_F2, delta_F2_rate, trailer_roll, trailer_roll_rate, 
                        velocidad, ruta, current_idx, dt, integrador_error, x_hat, 
                        min_lookahead=5.0, max_lookahead=15.0, k_lookahead_gain=3.5):
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

    # 2. Lookahead dinámico adaptativo para sacar el punto objetivo direccional (como transitorio LQR espacial)
    lookahead = np.clip(min_lookahead + (k_lookahead_gain * abs(velocidad)), min_lookahead, max_lookahead)
    target_idx = len(ruta) - 1
    found_target = False
    for i in range(closest_idx, len(ruta)):
        pt = ruta[i]
        dist = math.hypot(pos_trailer[0] - pt[0], pos_trailer[1] - pt[1])
        if dist >= lookahead:
            target_idx = i
            found_target = True
            break
            
    target_pt = ruta[target_idx]
    if not found_target and len(ruta) > 1:
        pt_last = ruta[-1]
        pt_prev = ruta[-2]
        dir_x = pt_last[0] - pt_prev[0]
        dir_y = pt_last[1] - pt_prev[1]
        norm = math.hypot(dir_x, dir_y)
        if norm > 0:
            dir_x /= norm
            dir_y /= norm
        target_pt = (pos_trailer[0] + dir_x * lookahead, pos_trailer[1] + dir_y * lookahead)

    # 3. Cálculo de Errores Verdaderos para el modelo LQR
    # A. Error angular (x2)
    dx = target_pt[0] - pos_trailer[0]
    dy = target_pt[1] - pos_trailer[1]
    yaw_objetivo = math.atan2(dy, dx)
    yaw_objetivo_reversa = (yaw_objetivo + math.pi) % (2 * math.pi) - math.pi
    
    # Error direccional del tráiler respecto a la meta (yaw_trailer)
    x2_error_angular = (yaw_objetivo_reversa - psi_trailer + math.pi) % (2 * math.pi) - math.pi

    # B. Error lateral (x1 o "cross track error") aproximado usando trigonometría simple
    # dist * sin(yaw_error) es la distancia perpendicular a la trayectoria ideal
    x1_error_lateral = min_dist * math.sin(x2_error_angular)

    # C. Águlo de articulación (x3)
    x3_delta_F2 = delta_F2
    
    # D. Acumulador integral (x4)
    integrador_error += x1_error_lateral * dt
    # Limitamos la cuerda del integrador para evitar Wind-up catastrófico
    integrador_error = np.clip(integrador_error, -5.0, 5.0)

    velocidad_abs = abs(velocidad)

    # Vector de Estados Medidos (Físicos Directos o Aproximados por Cámara para el observador)
    y_medido = np.array([[x1_error_lateral]]) # El observador (L_obs) del script de MATLAB mide C = [1,0,0], asume que medimos solo error lateral cross-track.

    # Cálculo dinámico basado en la matriz de Riccati (Computado On-The-Fly desde base de MATLAB modelaje.m)
    # D0 (distancia enganche a caja) la dejamos constante en 5.0 para el modelo nominal
    K_mimo, L_obs = calcular_matrices_lqr_obs(velocidad_abs, 5.0)

    # 4. Implementación del OBSERVADOR DUAL (Filtro de Luenberger sobre estados Físicos)
    # Extraemos el sub-estado físico (los 3 primeros elementos de x_hat)
    x_hat_fisico = x_hat[0:3, :]
    
    # Modelado dinámico numérico (Euler Forward Continuous-to-Discrete) usando matrices dinámicas
    v_eff = velocidad_abs if velocidad_abs > 0.1 else 0.1
    A_bwd_dyn = np.array([
        [0, -v_eff, 0],
        [0, 0, -v_eff/5.0],
        [0, 0, v_eff/5.0]
    ])
    B_bwd_dyn = np.array([[0], [0], [-v_eff]])
    C_nominal = np.array([[1, 0, 0]])

    # Predecimos la derivada del estado estimado (x_dot = A*x_hat + B*u + L*(y - C*x_hat))
    # Para el "u" usamos la acción calculada en el paso anterior, pero aquí simplificamos a 0 asumiendo actualización causal o u_previa
    # Para mayor rigurosidad, el error de innovación e_y = y_medido - y_estimado:
    y_estimado = C_nominal @ x_hat_fisico
    x_dot_estimado = A_bwd_dyn @ x_hat_fisico + L_obs @ (y_medido - y_estimado)
    
    x_hat_fisico = x_hat_fisico + (x_dot_estimado * dt)
    
    # Reensamblamos el vector del sistema MIMO completo de orden 4 agregando el integrador que controlamos de manera limpia, sin ruido.
    x_hat[0:3, :] = x_hat_fisico
    x_hat[3, 0] = integrador_error
    
    # 5. Ganancias MIMO y Ejecución LQR
    # Cálculo formal óptimo: u = -K * x 
    steering_raw = - float(K_mimo @ x_hat)
    
    # PREVENCIÓN EXTRA: Sistema de Suspensión Dinámica (8-DOF Active Control Anti-Roll)
    limite_roll = 0.08
    limite_roll_rate = 0.15
    atenuacion_dinamica = 1.0
    if abs(trailer_roll) > limite_roll or abs(trailer_roll_rate) > limite_roll_rate:
        exceso = max(abs(trailer_roll) / limite_roll, abs(trailer_roll_rate) / limite_roll_rate)
        atenuacion_dinamica = np.clip(1.0 / exceso, 0.4, 1.0)

    # Castigamos u también en el modelo LQR porque la dinámica "Soft Body" no está tabulada en A y B.
    steering_raw *= atenuacion_dinamica
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

    # Añadimos la respuesta direccional B*u al estado estimado actual para la próxima iteración.
    # Sabemos que B solo afecta fuertemente la variable física x3 (y derivadamente el resto en el siguiente dt)
    x_hat[0:3, :] += B_bwd_dyn * steering_raw * dt

    return steering, throttle, brake, closest_idx, False, integrador_error, x_hat


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
    
    # Estructura del Estado Estimado (Error lateral, Error angular, Articulación, Error Integral)
    x_hat = np.zeros((4, 1))
    integrador_error = 0.0

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

        # Tasas de cambio (no se integran directo en LQR A/B, pero se guardan por seguridad de atenuación dinámica de masas y roll mode)
        delta_F2_rate = (delta_F2 - previous_delta_F2) / dt
        trailer_roll_rate = (trailer_roll - previous_trailer_roll) / dt
        
        previous_delta_F2 = delta_F2
        previous_trailer_roll = trailer_roll
        last_time = current_time

        # Control Estado LQR (MIMO + Observador)
        steering, throttle, brake, current_idx, finished, integrador_error, x_hat = control_lqr_reversa(
            pos_trailer, psi_trailer, delta_F2, delta_F2_rate, trailer_roll, trailer_roll_rate, 
            velocidad, route_points, current_idx, dt, integrador_error, x_hat
        )

        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)

        if finished:
            print("Ruta de reversa LQR completada. Frenando y devolviendo control manual (en Neutro)...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            break

    input('Presione enter cuando termine la simulación en reversa (MIMO-LQR)...')
    # truck_trailer.bng.close()

if __name__ == "__main__":
    main()