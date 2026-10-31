import math
from arx_truck_ai import bng_open
import numpy as np
import scipy.linalg as la
from arx_truck_ai.debugging_sys import (registrar_muestra, generar_grafico_evaluacion, inicializar_telemetria,
                                       finalizar_telemetria)
from arx_truck_ai import truck_trailer as tt
from arx_truck_ai import fin_ruta as fr
from arx_truck_ai.fin_ruta import (distancia_restante, velocidad_limite_fin, debe_frenar,
                                   indice_segmento, detener_en_fin, ControlVelocidad)
from arx_truck_ai.geometria_ruta import curvatura_adelante

# ---------------------------------------------------------------------------
# Fin de ruta (ver fin_ruta.py). El punto que debe detenerse en el final es trailer_pos.
V_CRUCERO = 2.0          # m/s   velocidad de crucero en reversa
A_APROXIMACION = 0.3     # m/s²  desaceleración del perfil de llegada: v_obj = sqrt(2*a*d_restante)
V_LLEGADA = 0.5          # m/s   velocidad mínima del perfil mientras quede ruta
A_FRENADO = 1.5          # m/s²  desaceleración que se ASUME con freno a fondo (conservadora)
MARGEN_PARADA = 0.3      # m     margen extra antes del punto final
KP_VELOCIDAD = 0.4       # ganancia proporcional del control de velocidad (fin_ruta.ControlVelocidad)
# ---------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# MODELO DE DISEÑO DEL LQR
#
# Modelo FÍSICO del camión+tráiler en reversa, identificado a partir de la telemetría
# real de BeamNG (reproduce delta_F2, e_ct y e_trailer de una corrida real con errores de
# pocos %). Estados medidos: x1 = error lateral [m] (positivo a la izquierda de la ruta),
# x2 = error angular del tráiler [rad], x3 = delta_F2 = psi_truck - psi_trailer [rad],
# x4 = integral del error lateral [m*s] (integra +x1, igual que el código de control).
# Quinto estado: x5 = posición REAL del volante (el actuador responde con retraso); no se
# mide, se estima filtrando el propio comando anterior (previous_steering). La entrada del
# LQR es el comando de volante normalizado s en [-1, 1] (positivo = ruedas a la derecha).
#
#   x1_dot = -v*x2 + v*(DP/L2)*x3
#   x2_dot = +(v/L2)*x3                 <- el tráiler gira en sentido contrario al de la articulación
#   x3_dot = +(v/L2)*x3 + (v*PHI/L1)*x5 <- INESTABLE en reversa (jackknife): polo en +v/L2
#   x4_dot = +x1
#   x5_dot = (s - x5)/TAU
#
# El modelo anterior (A[2,1]=+v/D0, A[2,2]=-v/D0, A[1,2]=-v/D0, B=-v) describía una planta
# ESTABLE y con el acople x2<-x3 de signo opuesto; la K resultante tenía signo contrario
# al necesario en e_ct y e_trailer, y aplicada a la planta real daba un lazo cerrado
# inestable (polo en +0.44 1/s).
#
# Valores IDENTIFICADOS de la telemetría real: L1 = 2.93 m, L2 = 8.1 m, DP = 5.2 m, retardo del
# volante = lag de 0.34 s + retardo puro de 0.32 s (≈ 0.66 s en total).
# Valores de DISEÑO (deliberadamente PESIMISTAS: planta más inestable, volante menos eficaz,
# más retraso). Diseñar sobre la planta pesimista es lo que da robustez: con los valores
# identificados el diseño fallaba en ~3 % de plantas realistas; con estos, en ninguna.
# ----------------------------------------------------------------------------
L1_NOMINAL = 3.3             # m   ganancia del volante sobre la articulación: v*PHI/L1
L2_NOMINAL = 6.5             # m   quinta rueda -> eje del tráiler (inestabilidad: v/L2)
DP_NOMINAL = 5.2             # m   punto medido (trailer_pos) -> eje del tráiler
PHI_MAX_RAD = 0.6            # rad ángulo de rueda a steering = 1
TAU_VOLANTE_NOMINAL = 0.8    # s   retraso TOTAL del actuador de dirección que asume el estimador

# Puntos de operación (rad) para el gain scheduling sobre |delta_F2|.
PUNTOS_OPERACION_DELTA_F2 = (0.0, 0.15, 0.30, 0.45)

# Zona de transición para la penalización continua (rad).
ZONA_PELIGRO_INICIO = 0.10
ZONA_PELIGRO_FIN = 0.45

# Penalizaciones (en unidades físicas: m, rad, m*s, steering normalizado)
Q_X1 = 0.5                  # error lateral
Q_X2 = 5.0                  # error angular
Q_X4 = 0.05                 # integral del error lateral
Q_X5 = 0.5                  # posición real del volante
Q33_BASE = 0.5              # articulación en x3 = 0
Q33_MAX = 0.75              # articulación en la zona de peligro
R_MAX = 15.0                # penalización del esfuerzo de control en x3 = 0
R_MIN = 10.5                # ... en la zona de peligro (menor = contravolante más agresivo)
# NOTA sobre el scheduling: con el modelo físico correcto, hacerlo MÁS agresivo en la zona de
# peligro (p. ej. R x0.1, Q33 x20) EMPEORA la robustez (con retardo, más ganancia desestabiliza):
# en un barrido de condiciones extremas bajaba de 74 % a 60 % de corridas sin jackknife. Se
# mantiene el mecanismo en una versión suave (R x0.7, Q33 x1.5), prácticamente neutra.

# ----------------------------------------------------------------------------
# MEJORAS DE OCT-2026 (cada una se puede desactivar para comparar en BeamNG con la versión anterior)
#
# 1) PREALIMENTACIÓN DE CURVATURA. Sin ella el LQR regula hacia x = 0 (δF2 = 0, volante centrado) y en
#    una curva solo gira cuando ya hay error. Con ella regula la DESVIACIÓN respecto del estado de
#    régimen permanente que exige la curvatura kappa de la ruta (giro estacionario):
#        x3_ref = -atan(L2*kappa)              articulación de régimen (saturada en ±X3_REF_MAX_RAD)
#        x2_ref = (DP/L2)*x3_ref               error angular de régimen del punto medido (está a DP del eje)
#        u_ff   = -atan(L1*sin(x3_ref)/L2)/PHI  volante de régimen (x5_ref = u_ff)
#        u      = u_ff - K (x - x_ref)
#    (las relaciones linealizadas coinciden con hacer x_dot = 0 en el modelo de diseño con la tangente
#    de la ruta girando a v*kappa). kappa se toma T_PREVIEW_FF s por delante (compensa el retardo del volante).
#    Para el estado de régimen se usan los parámetros IDENTIFICADOS (mejor estimación de la planta), no
#    los pesimistas del diseño de K.
# 2) PROGRAMACIÓN DE GANANCIAS EN V. Además de |δF2|, la tabla de K se calcula para varias velocidades y se
#    interpola con la velocidad objetivo del perfil (ver abajo). Con K fija en V0 = 2 m/s el lazo queda
#    marginal a 0.5-1 m/s (tramo de llegada).
# 3) INTEGRADOR. 'original': integra x1 solo si |x1| < 3 cm y satura x4 en ±0.15 m·s (aporte al volante
#    < 0.01: en la práctica no hay acción integral). 'corregido': integra si |x1| < BANDA_INTEGRADOR_M,
#    congela la integración mientras el volante está saturado (anti-windup) y limita el aporte |K4·x4|
#    a LIMITE_ACCION_INTEGRAL. La K es la misma en ambos modos (Q_X4 no cambia).
# ----------------------------------------------------------------------------
PREALIMENTACION_CURVATURA = True
T_PREVIEW_FF = 0.8           # s   anticipación de la curvatura (≈ retardo del actuador)
X3_REF_MAX_RAD = 0.25        # rad articulación de régimen máxima que se pide (≈ 14°): en curvas más cerradas
                             #     que R = L2/tan(X3_REF_MAX) (≈ 32 m) se renuncia a seguir la curva exacta en
                             #     vez de pedir una articulación cercana al jackknife (la senoide de R = 13.5 m
                             #     exigiría 0.54 rad = 31°). Sin límite, en el arnés la senoide terminaba en
                             #     jackknife; con 0.25-0.35 rad no. Ajustable tras las pruebas en BeamNG.
L1_IDENT = 2.93              # m   parámetros identificados (estado de régimen de la prealimentación)
L2_IDENT = 8.1               # m
DP_IDENT = 5.2               # m

PROGRAMACION_EN_V = True
PUNTOS_OPERACION_V = (0.5, 1.0, 1.5, 2.0, 2.5)   # m/s
# La K se interpola con la velocidad OBJETIVO del perfil (V_CRUCERO y su reducción al llegar), no con la
# medida: al arrancar la velocidad medida es ~0 y las K de baja velocidad (más agresivas) se aplicarían
# justo cuando el error inicial es máximo (capturas de 90°); en el arnés eso subía la articulación máxima
# de 38° a 55°. La velocidad objetivo vale V_CRUCERO al arrancar y baja suavemente en la llegada.

MODO_INTEGRADOR = 'original'  # 'original' | 'corregido'
BANDA_INTEGRADOR_M = 0.5
LIMITE_ACCION_INTEGRAL = 0.15  # aporte máximo del integrador al volante normalizado


def _factor_suavizado(x3_abs, inicio=ZONA_PELIGRO_INICIO, fin=ZONA_PELIGRO_FIN):
    """
    Smoothstep (C1 continuo, derivada nula en los extremos) que mapea |x3| -> [0, 1]:
    0 por debajo de 'inicio' (zona segura), 1 por encima de 'fin' (zona de peligro).
    """
    s = np.clip((x3_abs - inicio) / (fin - inicio), 0.0, 1.0)
    return s * s * (3 - 2 * s)


def _R_lqr_continuo(x3_op):
    """R decrece suave de R_MAX a R_MIN al entrar en la zona de peligro."""
    s = _factor_suavizado(abs(x3_op))
    return R_MAX * (1 - s) + R_MIN * s


def _Q33_continuo(x3_op):
    """Q_33 crece de forma exponencial entre Q33_BASE y Q33_MAX con el mismo factor."""
    s = _factor_suavizado(abs(x3_op))
    return Q33_BASE * (Q33_MAX / Q33_BASE) ** s


def _construir_modelo(v_eff, L2, L1=None, dp=None, tau=None):
    """Matrices (A, B) del modelo de diseño de 5 estados [x1, x2, x3, x4, x5] (ver arriba)."""
    L1 = L1_NOMINAL if L1 is None else L1
    dp = DP_NOMINAL if dp is None else dp
    tau = TAU_VOLANTE_NOMINAL if tau is None else tau
    A = np.zeros((5, 5))
    A[0, 1] = -v_eff
    A[0, 2] = v_eff * dp / L2
    A[1, 2] = v_eff / L2
    A[2, 2] = v_eff / L2
    A[2, 4] = v_eff * PHI_MAX_RAD / L1
    A[3, 0] = 1.0
    A[4, 4] = -1.0 / tau
    B = np.zeros((5, 1))
    B[4, 0] = 1.0 / tau
    return A, B


def calcular_K_punto_operacion(V0, D0, x3_op):
    """
    Resuelve la LQR (CARE) para un punto de operación x3_op y devuelve K (1x5):
        steering = -K @ [x1, x2, x3, x4, x5]
    D0 es la longitud del tráiler L2 [m] (antes era un parámetro calibrado a mano).
    """
    v_eff = V0 if abs(V0) > 0.1 else 0.1
    A, B = _construir_modelo(v_eff, D0)
    Q_lqr = np.diag([Q_X1, Q_X2, _Q33_continuo(x3_op), Q_X4, Q_X5])
    R_lqr = np.array([[_R_lqr_continuo(x3_op)]])
    X_lqr = la.solve_continuous_are(A, B, Q_lqr, R_lqr)
    return np.linalg.inv(R_lqr) @ B.T @ X_lqr


def precalcular_ganancias_lqr(V0_nominal, D0_nominal,
                               puntos_operacion=PUNTOS_OPERACION_DELTA_F2, velocidades=None):
    """
    Precalcula UNA SOLA VEZ (fuera del bucle de control) las matrices K en cada punto de operación
    de articulación |δF2| (y, si `velocidades` no es None, en cada velocidad). El bucle solo interpola.

    Sin `velocidades` la tabla es 1D y todas las K se calculan en V0_nominal (comportamiento hasta
    oct-2026). Con `velocidades` la tabla es 2D (V x |δF2|) y se interpola con la velocidad medida:
    con una K fija en V0 = 2 m/s el lazo cerrado sobre el modelo identificado queda marginal a
    0.5-1 m/s (tramo de llegada). D0_nominal es L2 del modelo de diseño.
    (El análisis de incertidumbre de matlab/modelaje.m usa el modelo nominal de 3 estados, no este.)

    Cada K incorpora la variación continua de R y Q_33 con |δF2| (ver _R_lqr_continuo / _Q33_continuo).
    Devuelve un dict con 'puntos_operacion', 'velocidades' (None o tupla) y 'Ks'
    (lista de K, o lista de listas [i_v][i_op] si hay velocidades).
    """
    if velocidades is None:
        Ks = [calcular_K_punto_operacion(V0_nominal, D0_nominal, op) for op in puntos_operacion]
    else:
        Ks = [[calcular_K_punto_operacion(v, D0_nominal, op) for op in puntos_operacion] for v in velocidades]
    return {
        'puntos_operacion': puntos_operacion,
        'velocidades': None if velocidades is None else tuple(velocidades),
        'Ks': Ks,
    }


def _interpolar_1d(x, puntos, valores):
    """Interpolación lineal de matrices `valores` sobre `puntos` crecientes (saturada en los extremos)."""
    x = min(max(x, puntos[0]), puntos[-1])
    for i in range(len(puntos) - 1):
        x0, x1 = puntos[i], puntos[i + 1]
        if x0 <= x <= x1:
            w = (x - x0) / (x1 - x0) if (x1 - x0) > 1e-9 else 0.0
            return (1 - w) * valores[i] + w * valores[i + 1]
    return valores[-1]


def interpolar_ganancia_lqr(x3_delta_F2, tabla_ganancias, velocidad=None):
    """
    Interpola linealmente (sin resolver Riccati) la ganancia K según |δF2| y, si la tabla tiene
    velocidades, también según |velocidad| (interpolación bilineal, saturada en los extremos).
    """
    puntos_operacion = tabla_ganancias['puntos_operacion']
    Ks = tabla_ganancias['Ks']
    velocidades = tabla_ganancias.get('velocidades')
    x3_abs = abs(x3_delta_F2)
    if velocidades is None or velocidad is None:
        fila = Ks if velocidades is None else Ks[-1]
        return _interpolar_1d(x3_abs, puntos_operacion, fila)
    por_velocidad = [_interpolar_1d(x3_abs, puntos_operacion, fila) for fila in Ks]
    return _interpolar_1d(abs(velocidad), velocidades, por_velocidad)


def estado_regimen_curva(kappa):
    """
    Estado de régimen permanente (x_ref, u_ff) para seguir una curva de curvatura `kappa` [1/m] en
    reversa (ver MEJORAS DE OCT-2026, punto 1). x_ref = [0, x2_ref, x3_ref, 0, u_ff].
    """
    x3_ref = -float(np.clip(math.atan(L2_IDENT * kappa), -X3_REF_MAX_RAD, X3_REF_MAX_RAD))
    x2_ref = (DP_IDENT / L2_IDENT) * x3_ref
    u_ff = -math.atan(L1_IDENT * math.sin(x3_ref) / L2_IDENT) / PHI_MAX_RAD
    return np.array([[0.0], [x2_ref], [x3_ref], [0.0], [u_ff]]), u_ff


def control_lqr_reversa(pos_trailer, psi_trailer, delta_F2, trailer_roll, trailer_roll_rate,
                        velocidad, ruta, current_idx, dt, integrador_error, x_hat, previous_steering,
                        tabla_ganancias=None, ctrl_vel=None):
    """
    Control LQR por realimentación de estados con programación de ganancias sobre |delta_F2| (y |v|),
    con prealimentación de curvatura opcional (ver MEJORAS DE OCT-2026).
    x1..x4 se miden; x5 se estima con el modelo del actuador (no hay observador de Luenberger).
    Retorna (steering, throttle, brake, next_idx, finished, nuevo_integrador, x_hat_nuevo,
             x1_error_lateral, x2_error_angular, x3_delta_F2, steering_raw).

    Estados (ver MODELO DE DISEÑO DEL LQR al inicio del archivo):
    x1: Error lateral del punto medido del remolque [m]
    x2: Error angular del remolque respecto a la tangente de la ruta [rad]
    x3: Ángulo de articulación delta_F2 [rad]
    x4: Integral del error lateral [m*s] (ver MODO_INTEGRADOR)
    x5: Posición real estimada del volante (comando normalizado)

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
        return 0.0, 0.0, 1.0, current_idx, True, integrador_error, x_hat, 0.0, 0.0, 0.0, 0.0

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

    # Distancia que falta hasta el final, medida A LO LARGO de la ruta (negativa si ya se pasó).
    # El fin de ruta se decide al final de la función (ver debe_frenar), no aquí: antes se exigía
    # que el tráiler estuviera a < 2 m del último punto, y si llegaba desviado nunca se cumplía.
    d_restante = distancia_restante(ruta, closest_idx, pos_trailer)
    
    # B. Error lateral (x1 o "cross track error") 
    # Proyección ortogonal directa de la distancia entre el tráiler y el segmento de la ruta para evitar zonas muertas angulares.
    # En el último punto se usa el último segmento: con el punto final repetido, el segmento tenía
    # longitud 0 y atan2(0, 0) = 0 daba un rumbo de referencia falso (el error angular x2 saltaba al
    # rumbo absoluto del tráiler en rutas curvas y el LQR terminaba en jackknife).
    idx_seg = indice_segmento(ruta, closest_idx)
    pt_actual_tr = ruta[idx_seg]
    pt_siguiente_tr = ruta[idx_seg + 1]
    dx_path = pt_siguiente_tr[0] - pt_actual_tr[0]
    dy_path = pt_siguiente_tr[1] - pt_actual_tr[1]
    yaw_path = math.atan2(dy_path, dx_path)
    dx_tr = pos_trailer[0] - pt_actual_tr[0]
    dy_tr = pos_trailer[1] - pt_actual_tr[1]
    x1_error_lateral = dy_tr * math.cos(yaw_path) - dx_tr * math.sin(yaw_path)

    # 2. Errores para el modelo LQR (el LQR no usa punto de lookahead)

    # A. Error angular (x2): alinear el tráiler con la dirección del segmento de ruta
    psi_trailer_reversa = psi_trailer + math.pi  # el tráiler se mueve hacia atrás
    x2_error_angular = math.atan2(
        math.sin(yaw_path - psi_trailer_reversa),
        math.cos(yaw_path - psi_trailer_reversa)
    )

    # C. Articulación
    x3_delta_F2 = np.clip(delta_F2, -1.1, 1.1)

    # La ganancia sale de interpolar la tabla precalculada (sin Riccati aquí): en |δF2| y, si la
    # tabla tiene velocidades (PROGRAMACION_EN_V), también en la velocidad objetivo del perfil.
    v_programacion = min(V_CRUCERO, velocidad_limite_fin(d_restante, A_APROXIMACION, V_LLEGADA))
    K_mimo = interpolar_ganancia_lqr(x3_delta_F2, tabla_ganancias, v_programacion)

    # D. Acumulador integral (x4), ver MEJORAS DE OCT-2026 punto 3
    if MODO_INTEGRADOR == 'corregido':
        volante_saturado = abs(previous_steering) >= 0.999
        if abs(x1_error_lateral) < BANDA_INTEGRADOR_M and not volante_saturado:
            integrador_error += x1_error_lateral * dt
        k4 = abs(float(K_mimo[0, 3]))
        limite_x4 = LIMITE_ACCION_INTEGRAL / k4 if k4 > 1e-9 else 0.0
        integrador_error = float(np.clip(integrador_error, -limite_x4, limite_x4))
    else:
        if abs(x1_error_lateral) < 0.03:
            integrador_error += x1_error_lateral * dt
        # Limitamos la cuerda del integrador para evitar Wind-up catastrófico
        integrador_error = np.clip(integrador_error, -0.15, 0.15)

    # E. Prealimentación de curvatura (MEJORAS DE OCT-2026 punto 1): estado de régimen de la curva
    kappa = 0.0
    if PREALIMENTACION_CURVATURA:
        kappa = curvatura_adelante(ruta, closest_idx, pos_trailer, adelanto_m=abs(velocidad) * T_PREVIEW_FF)
    x_ref, u_ff = estado_regimen_curva(kappa)

    # x1..x4 se miden directamente. El 5.º estado (posición REAL del volante, que va
    # retrasada respecto al comando) se estima filtrando el comando anterior con la
    # constante de tiempo del actuador: es un modelo de la planta, no un observador de
    # estados.

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
    # Estimación de x5: discretización exacta del retardo de primer orden con comando constante
    # (previous_steering) durante el último intervalo dt.
    alfa_volante = 1.0 - math.exp(-max(dt, 1e-4) / TAU_VOLANTE_NOMINAL)
    x_hat[4,0] += alfa_volante * (previous_steering - x_hat[4,0])
    
    # 3. Ganancias MIMO y Ejecución LQR
    # Cálculo formal óptimo: u = u_ff - K (x - x_ref), con u = comando de volante normalizado [-1, 1].
    # Sin prealimentación (o en tramo recto) x_ref = 0 y u_ff = 0: u = -K x.
    # (.item(): compatible con NumPy>=2.4)
    steering_raw = u_ff - float((K_mimo @ (x_hat - x_ref)).item())
    
    # PREVENCIÓN EXTRA: atenuación del volante por balanceo (roll) del remolque
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
    
    # Bucle de velocidad básico, con perfil de llegada: cerca del final la velocidad objetivo baja
    # como sqrt(2*a*d_restante) para llegar despacio al punto final.
    velocidad_objetivo = min(V_CRUCERO, velocidad_limite_fin(d_restante, A_APROXIMACION, V_LLEGADA)) * atenuacion_dinamica
    # Control de velocidad común (fin_ruta.ControlVelocidad: PI o la ley 'original')
    if ctrl_vel is None:
        ctrl_vel = ControlVelocidad(KP_VELOCIDAD, V_CRUCERO, modo='original')
    throttle, brake = ctrl_vel(velocidad_objetivo, velocidad, dt)

    # Fin de ruta: frenar a fondo cuando la distancia de parada alcanza a la que queda (o si ya se pasó).
    if debe_frenar(d_restante, velocidad, A_FRENADO, MARGEN_PARADA):
        return steering, 0.0, 1.0, closest_idx, True, integrador_error, x_hat, x1_error_lateral, x2_error_angular, x3_delta_F2, steering_raw

    return steering, throttle, brake, closest_idx, False, integrador_error, x_hat, x1_error_lateral, x2_error_angular, x3_delta_F2, steering_raw


def main():
    direction = 'bwd'
    truck_trailer, orig = bng_open.main(direction)

    # Al poner la caja de cambios del camión en Reversa (Gear = -1), el acelerador lo desplazará hacia atrás
    truck_trailer.iniciar_simulacion(gear=-1)

    current_idx = 0
    route_points = truck_trailer.route
    nombre_ruta = truck_trailer.nombre_ruta

    # Inicialización de estado para LQR (Integrador, estimación del volante y tasa de roll)
    previous_trailer_roll = None   # None: tasa de roll 0 en el primer ciclo (no se deriva contra 0)
    previous_steering = 0.0
    
    # Estructura del estado (Error lateral, Error angular, Articulación, Error Integral,
    # Posición estimada del volante)
    x_hat = np.zeros((5, 1))
    integrador_error = 0.0

    # D0_NOMINAL es la longitud del tráiler L2 del modelo de diseño (L2_NOMINAL).
    D0_NOMINAL = L2_NOMINAL

    # Velocidad de DISEÑO (no la instantánea), igual a velocidad_objetivo del lazo de velocidad.
    # Las K se precalculan aquí, UNA sola vez, antes de entrar al bucle de control.
    V0_NOMINAL = 2.0
    tabla_ganancias = precalcular_ganancias_lqr(
        V0_NOMINAL, D0_NOMINAL, velocidades=PUNTOS_OPERACION_V if PROGRAMACION_EN_V else None)

    primer_tick = True

    # Telemetría (entrada de evaluacion_resultados.py): una fila por paso de simulación
    inicializar_telemetria(f"telemetria_lqr_{nombre_ruta}.csv", puntos_ruta=route_points, sentido=-1,
                           punto_control='trailer', algoritmo='lqr', ruta=nombre_ruta,
                           tipo_ruta=tt.TIPO_RUTA, modo='sincrono' if tt.MODO_SINCRONO else 'libre',
                           frecuencia_simulacion=tt.FRECUENCIA_SIMULACION,
                           frecuencia_control=tt.FRECUENCIA_SIMULACION / truck_trailer.decimacion_control
                           if tt.MODO_SINCRONO else None,
                           prealimentacion_curvatura=PREALIMENTACION_CURVATURA,
                           programacion_en_v=PROGRAMACION_EN_V, modo_integrador=MODO_INTEGRADOR,
                           control_velocidad=fr.MODO_CONTROL_VELOCIDAD)

    t0 = None
    t_ultimo_control = None
    steering, throttle, brake = 0.0, 0.0, 0.0
    ctrl_vel = ControlVelocidad(KP_VELOCIDAD, V_CRUCERO)
    x1 = x2 = None
    finished = False
    resultado, info_fin = 'completada', {}
    ciclo = 0

    while True:
        # En modo síncrono el control se recalcula cada `decimacion_control` pasos y la telemetría se
        # registra en todos; en modo libre ambos ocurren en cada iteración.
        es_ciclo_control = (ciclo % truck_trailer.decimacion_control == 0)
        sensores = truck_trailer.read_sensors(is_reverse=True, completo=es_ciclo_control)
        t = sensores['t_sim']
        if t0 is None:
            t0 = t
        t_rel = t - t0

        velocidad = sensores["v1"]
        psi_trailer = sensores["psi_trailer"]
        delta_F2 = sensores["delta_F2"]
        pos_trailer = sensores["trailer_pos"]
        trailer_roll = sensores["trailer_roll"]

        if primer_tick:
            print(f"[D0-DIAG] D0_NOMINAL={D0_NOMINAL} | delta_F2_inicial={delta_F2:.4f} rad | velocidad={velocidad:.3f} m/s")
            print(f"[D0-DIAG] pos_trailer={pos_trailer}")
            primer_tick = False

        if es_ciclo_control:
            dt = (t - t_ultimo_control) if t_ultimo_control is not None else (truck_trailer.dt_control or 0.1)
            dt = max(dt, 1e-4)
            t_ultimo_control = t

            # Tasa de roll (no está en A/B; se usa en la atenuación dinámica de roll)
            trailer_roll_rate = 0.0 if previous_trailer_roll is None else (trailer_roll - previous_trailer_roll) / dt
            previous_trailer_roll = trailer_roll

            # Control Estado LQR (MIMO, Gain Scheduled)
            steering, throttle, brake, current_idx, finished, integrador_error, x_hat, x1, x2, x3, s_raw = control_lqr_reversa(
                pos_trailer, psi_trailer, delta_F2, trailer_roll, trailer_roll_rate,
                velocidad, route_points, current_idx, dt, integrador_error, x_hat, previous_steering=previous_steering,
                tabla_ganancias=tabla_ganancias, ctrl_vel=ctrl_vel
            )
            truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)
            previous_steering = steering

        # Telemetría: se registra `steering` (comando aplicado, saturado a [-1, 1]); el tick de fin de
        # ruta es un comando de frenado, no una muestra de seguimiento.
        if not finished:
            registrar_muestra(t_rel, sensores, steering, control_actualizado=es_ciclo_control,
                              error_lateral_ctrl=x1, error_angular_ctrl=x2,
                              throttle=throttle, brake=brake)

        if finished:
            print("Fin de ruta (LQR): frenando hasta detener el conjunto...")
            # Frena hasta detenerse y deja el camión con freno de mano, sin freno de servicio y en neutro.
            sobrepaso, d_fin = detener_en_fin(truck_trailer, route_points, current_idx, 'trailer_pos',
                                              steering=steering, is_reverse=True, gear=-1)
            print(f"Conjunto detenido. Sobrepaso del tráiler a lo largo de la ruta: {sobrepaso:+.2f} m "
                  f"(+ = se pasó del punto final) | distancia al punto final: {d_fin:.2f} m")
            resultado, info_fin = 'completada', dict(sobrepaso_m=sobrepaso, distancia_final_m=d_fin)
            break
        elif abs(delta_F2) > 1.0: # Condición de Jackknife (aprox 57 grados)
            print(f"¡Jackknife inminente detectado! (Articulación: {delta_F2:.2f} rad). Abortando para graficar...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            resultado, info_fin = 'jackknife', dict(delta_F2_final_rad=float(delta_F2))
            break
        elif t_rel > tt.DURACION_MAXIMA_S:
            print(f"Tiempo máximo del experimento ({tt.DURACION_MAXIMA_S:.0f} s) alcanzado: deteniendo...")
            sobrepaso, d_fin = detener_en_fin(truck_trailer, route_points, current_idx, 'trailer_pos',
                                              steering=steering, is_reverse=True, gear=-1)
            resultado, info_fin = 'tiempo_maximo', dict(sobrepaso_m=sobrepaso, distancia_final_m=d_fin)
            break

        truck_trailer.avanzar()      # modo síncrono: un paso de simulación (1/60 s); modo libre: nada
        ciclo += 1

    finalizar_telemetria(resultado, **info_fin)
    truck_trailer.liberar_simulacion()
    # Cuando salimos del bucle, graficamos
    generar_grafico_evaluacion()
    input('Presione enter cuando termine la simulación en reversa (MIMO-LQR)...')

if __name__ == "__main__":
    main()
