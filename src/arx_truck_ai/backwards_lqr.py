import math
from arx_truck_ai import bng_open
from beamngpy import Vehicle
import numpy as np
import scipy.linalg as la
import time
from arx_truck_ai.debugging_sys import registrar_estado, generar_grafico_evaluacion

# Puntos de operación (rad) para el gain scheduling sobre |delta_F2|.
# Deben ir de menor a mayor. Cubren desde el equilibrio hasta la zona de
# peligro de jackknife, con resolución suficiente para que la interpolación
# capture la variación continua de R y Q_33.
PUNTOS_OPERACION_DELTA_F2 = (0.0, 0.15, 0.30, 0.45)

# Zona de transición para la penalización continua (rad). Antes de
# ZONA_PELIGRO_INICIO el comportamiento es el nominal "suave"; después de
# ZONA_PELIGRO_FIN se satura en el extremo "agresivo".
# Recalibrado: arranca antes (0.10 en vez de 0.15) para que la corrección
# sea gradual desde más temprano y no golpee el esfuerzo de control de golpe.
ZONA_PELIGRO_INICIO = 0.10
ZONA_PELIGRO_FIN = 0.45

# Extremos de penalización del esfuerzo de control (R) y de la articulación (Q_33)
# Recalibrado tras observar saturación severa del volante (-2.0 a +3.5 contra
# el límite físico [-1,1]): se reduce mucho la agresividad de ambos extremos.
R_MAX = 1500.0    # en x3=0: comportamiento suave, igual que antes
R_MIN = 1400.0    # con este en 1800 y R_MAX en 5000 dura 20 segundos
Q33_BASE = 8000.0   # en x3=0: igual que antes
Q33_MAX = 18000.0   # antes 40000 -> demasiado agresivo, saturaba el actuador


def _factor_suavizado(x3_abs, inicio=ZONA_PELIGRO_INICIO, fin=ZONA_PELIGRO_FIN):
    """
    Smoothstep (C1 continuo, derivada nula en los extremos) que mapea
    |x3| -> [0, 1]: 0 por debajo de 'inicio' (zona segura), 1 por encima
    de 'fin' (zona de peligro saturada), transición suave en el medio.
    Evita saltos bruscos de ganancia (que un sensor ruidoso convertiría
    en chattering) frente al antiguo umbral duro en 0.03 rad.
    """
    s = np.clip((x3_abs - inicio) / (fin - inicio), 0.0, 1.0)
    return s * s * (3 - 2 * s)


def _R_lqr_continuo(x3_op):
    """
    R decrece suave y progresivamente de R_MAX a R_MIN a medida que |x3_op|
    se acerca/entra en la zona de peligro: menos penalización del esfuerzo
    de control => contravolante más agresivo disponible para recuperar el
    jackknife.
    """
    s = _factor_suavizado(abs(x3_op))
    return R_MAX * (1 - s) + R_MIN * s


def _Q33_continuo(x3_op):
    """
    Q_33 (penalización de la articulación) crece de forma EXPONENCIAL entre
    Q33_BASE y Q33_MAX según el mismo factor de suavizado que R, para que el
    LQR castigue cada vez más fuerte cualquier desviación de x3 a medida que
    se acerca a la zona de peligro.
    """
    s = _factor_suavizado(abs(x3_op))
    return Q33_BASE * (Q33_MAX / Q33_BASE) ** s


def _construir_A_bwd(v_eff, D0):
    """
    Matriz de estado en retroceso — forma CALIBRADA PARA BEAMNG.DRIVE
    (confirmada intencional, distinta de la forma pura de modelaje.m).

    A diferencia del modelo nominal de modelaje.m (A[2,1]=0, A[2,2]=+V0/D0),
    aquí se usa A[2,1]=+v_eff/D0 y A[2,2]=-v_eff/D0: el término extra y el
    cambio de signo compensan convenciones de coordenadas y dinámicas del
    motor de físicas de BeamNG que no están representadas en el modelo
    analítico puro. Esta es la forma que debe usarse para el controlador
    en ejecución; modelaje.m sigue siendo la referencia de diseño/análisis
    (Bode, loop shaping, robustez), no la forma exacta de simulación.
    """
    return np.array([
        [0,      -v_eff,        0        ],
        [0,       0,        -v_eff/D0    ],
        [0,   v_eff/D0,      -v_eff/D0    ]
    ])


def calcular_K_punto_operacion(V0, D0, x3_op):
    """
    Resuelve la LQR (CARE) para un único punto de operación x3_op y devuelve
    la ganancia aumentada K_aug (1x4). No hay observador: se asume
    instrumentación completa de los 4 estados (medición directa).

    A_bwd/B_bwd no dependen de x3_op (calibración BeamNG fija). La variación
    entre puntos de operación viene de R_lqr y Q_33, ahora funciones
    CONTINUAS de |x3_op| (ver _R_lqr_continuo / _Q33_continuo) en vez del
    salto binario anterior: R baja suavemente y Q_33 sube exponencialmente
    a medida que la articulación se acerca a la zona de peligro de
    jackknife.
    """
    v_eff = V0 if abs(V0) > 0.1 else 0.1

    A_bwd = _construir_A_bwd(v_eff, D0)

    B_bwd = np.array([
        [0],
        [0],
        [-v_eff]
    ])

    C = np.array([[1, 0, 0]])

    # Modelo aumentado con estado extra de error integral
    A_aug = np.zeros((4, 4))
    A_aug[0:3, 0:3] = A_bwd
    A_aug[3, 0:3] = -C

    B_aug = np.zeros((4, 1))
    B_aug[0:3, :] = B_bwd

    Q_lqr = np.diag([
        5000,                    # error lateral
        4000,                    # error angular (antes 100 -- demasiado bajo para un sistema inestable)
        _Q33_continuo(x3_op),    # articulación (crece exponencialmente cerca del jackknife)
        100                      # integrador
    ])

    R_lqr = np.array([[_R_lqr_continuo(x3_op)]])

    X_lqr = la.solve_continuous_are(A_aug, B_aug, Q_lqr, R_lqr)
    K_aug = np.linalg.inv(R_lqr) @ B_aug.T @ X_lqr

    return K_aug


def precalcular_ganancias_lqr(V0_nominal, D0_nominal,
                               puntos_operacion=PUNTOS_OPERACION_DELTA_F2):
    """
    Precalcula UNA SOLA VEZ (fuera del bucle de control) las matrices K en
    cada punto de operación de articulación. Así se evita resolver la CARE
    en cada tick a 60Hz; el bucle de control solo interpola.

    V0_nominal / D0_nominal son el punto de DISEÑO fijo, no la velocidad
    instantánea del camión. Esto es coherente con el análisis de
    incertidumbre parametrica (+-10% de V0 y D0) que ya hiciste en
    modelaje.m: esa validación de robustez es justo lo que permite fijar
    la ganancia en un punto nominal sin recalcularla en tiempo real. Si en
    tus pruebas la velocidad real se aleja de ese +-10% ya validado,
    conviene volver a correr el análisis de Bode/loop-shaping del .m con el
    nuevo rango, o extender esta tabla a una segunda dimensión sobre V.

    Cada K en la tabla ya incorpora la variación continua de R y Q_33 (ver
    _R_lqr_continuo / _Q33_continuo): al interpolar entre puntos de
    operación consecutivos se obtiene un efecto anti-jackknife progresivo
    de verdad, no una ganancia plana como cuando A no dependía de x3.

    Devuelve un dict con 'puntos_operacion' y 'Ks' (lista de matrices K en
    el mismo orden), listo para pasar a control_lqr_reversa.
    """
    Ks = [calcular_K_punto_operacion(V0_nominal, D0_nominal, op)
          for op in puntos_operacion]
    return {
        'puntos_operacion': puntos_operacion,
        'Ks': Ks,
    }


def interpolar_ganancia_lqr(x3_delta_F2, tabla_ganancias):
    """
    Interpola linealmente (sin resolver Riccati) la ganancia K entre los
    puntos de operación precalculados, según |x3_delta_F2| actual.
    """
    puntos_operacion = tabla_ganancias['puntos_operacion']
    Ks = tabla_ganancias['Ks']

    x3_abs = min(abs(x3_delta_F2), puntos_operacion[-1])

    for i in range(len(puntos_operacion) - 1):
        x0, x1 = puntos_operacion[i], puntos_operacion[i + 1]
        if x0 <= x3_abs <= x1:
            w = (x3_abs - x0) / (x1 - x0) if (x1 - x0) > 1e-9 else 0.0
            return (1 - w) * Ks[i] + w * Ks[i + 1]

    return Ks[-1]

def control_lqr_reversa(pos_trailer, psi_trailer, delta_F2, delta_F2_rate, trailer_roll, trailer_roll_rate, 
                        velocidad, ruta, current_idx, dt, integrador_error, x_hat, previous_steering,
                        min_lookahead=5.0, max_lookahead=15.0, k_lookahead_gain=3.5,
                        tabla_ganancias=None):
    """
    Control LQR Óptimo con variables de estado (Gain Scheduled, sin observador).
    Retorna (steering, throttle, brake, next_idx, finished, nuevo_integrador, x_hat_nuevo).

    Estados del controlador MIMO:
    x1: Error lateral (desviación espacial hacia la ruta)
    x2: Error angular (heading error del remolque)
    x3: Ángulo de articulación (delta_F2)
    x4: Integrador del error (chattering prevention)

    tabla_ganancias: dict devuelto por precalcular_ganancias_lqr(), calculado
    UNA SOLA VEZ antes del bucle de control. Aquí NO se resuelve ninguna
    Riccati; solo se interpola entre las K ya calculadas.
    """
    if tabla_ganancias is None:
        raise ValueError(
            "control_lqr_reversa requiere 'tabla_ganancias' precalculada "
            "con precalcular_ganancias_lqr() antes de entrar al bucle de control."
        )
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

    # Instrumentación completa: los 4 estados se miden/estiman directamente
    # (no hay observador). La ganancia sale de interpolar la tabla precalculada
    # (sin resolver Riccati aquí) según el ángulo de articulación actual.
    K_mimo = interpolar_ganancia_lqr(x3_delta_F2, tabla_ganancias)

    # Inyección directa de las lecturas físicas de los sensores de orientación y articulación
    # Evita que el sistema asuma estados lineales nulos cuando la dinámica entra en el régimen no lineal del efecto tijera

    # NOTA: se eliminó la zona muerta suave sobre x2 (peso_x2/x2_efectivo).
    # Enmascaraba el error angular cerca de x3=0, justo donde el LQR más
    # necesita verlo para corregir pequeñas desviaciones de heading antes de
    # que degeneren en jackknife (línea recta = régimen donde x2 es la
    # primera señal de alerta). Ahora se inyecta x2_error_angular puro.
    x_hat[0,0] = x1_error_lateral
    x_hat[1,0] = x2_error_angular
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
    direction = 'bwd'
    truck_trailer, orig = bng_open.main(direction)

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
    # Probar D0 = 3.0, 4.0, 5.0, 6.0, 7.0 cambiando este valor.
    D0_NOMINAL = 3.0  # ← cambiar este valor entre pruebas para calibrar D0
    # ──────────────────────────────────────────────────────────────────────────

    # Velocidad de DISEÑO (no la instantánea): coincide con el V0=2.0 de
    # modelaje.m y con velocidad_objetivo del lazo de velocidad más abajo.
    # Las K se precalculan aquí, UNA sola vez, antes de entrar al bucle a 60Hz.
    V0_NOMINAL = 2.0
    tabla_ganancias = precalcular_ganancias_lqr(V0_NOMINAL, D0_NOMINAL)

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

        # Control Estado LQR (MIMO, Gain Scheduled, sin observador)
        steering, throttle, brake, current_idx, finished, integrador_error, x_hat, x1, x2, x3, s_raw = control_lqr_reversa(
        pos_trailer, psi_trailer, delta_F2, delta_F2_rate_filtrada, trailer_roll, trailer_roll_rate, 
        velocidad, route_points, current_idx, dt, integrador_error, x_hat, previous_steering=previous_steering,
        min_lookahead=2.0, max_lookahead=8.0, k_lookahead_gain=1.0, tabla_ganancias=tabla_ganancias
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