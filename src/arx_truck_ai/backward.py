import math
from arx_truck_ai import bng_open
from arx_truck_ai.debugging_sys import registrar_estado, generar_grafico_evaluacion, generar_diagrama_ray_casting, inicializar_telemetria
from beamngpy import Vehicle
import numpy as np
import time


def _envolver_angulo(angulo):
    """
    Normaliza un ángulo (rad) a (-pi, pi] usando atan2(sin, cos), que
    garantiza el camino más corto y es numéricamente robusto frente al
    operador módulo con floats.
    """
    return math.atan2(math.sin(angulo), math.cos(angulo))


# Offset entre el rumbo hacia el objetivo (dirección de MOVIMIENTO) y el heading que
# reportan psi_trailer/psi_truck. truck_trailer.read_sensors calcula psi con
# arctan2(state['dir']), o sea, hacia dónde apunta la NARIZ del vehículo. En reversa la
# nariz mira al lado opuesto al sentido de marcha (en el escenario 'bwd': nariz a -x,
# ruta hacia +x), por lo que el setpoint de heading es yaw_objetivo + pi.
# Con 0.0 un camión perfectamente alineado produce e_trailer = ±pi.
OFFSET_HEADING_REVERSA = math.pi

# ----------------------------------------------------------------------------
# GANANCIAS DE LA LEY DE CONTROL
#
#   steering = -( K_TRAILER*e_trailer + K_TRUCK*e_truck
#                 + K_D_ARTICULACION*d(delta_F2)/dt + K_D_TRAILER*d(e_trailer)/dt )
#
# Como e_truck = e_trailer - delta_F2 (delta_F2 = psi_truck - psi_trailer), esto es:
#
#   steering = -K2*e_trailer - K3*delta_F2 - (términos derivativos)
#       K3 = -K_TRUCK              amortiguamiento de la ARTICULACIÓN (contra el jackknife)
#       K2 =  K_TRAILER + K_TRUCK  seguimiento de la TRAYECTORIA
#
# Convención de signos (verificada con un modelo físico identificado a partir de la
# telemetría real): en reversa la articulación es inestable (delta_dot ≈ +a*delta + g*steering,
# con g > 0), por lo que la realimentación de delta_F2 debe ser NEGATIVA (K3 > 0 => K_TRUCK < 0)
# y K3 debe superar a/g + (v/lookahead)/g (≈ 1 con los parámetros identificados).
# K_TRAILER debe ser MAYOR que |K_TRUCK| (K2 > 0). Con K_TRAILER < |K_TRUCK| la
# realimentación del error de trayectoria tiene signo contrario y el sistema diverge.
# ----------------------------------------------------------------------------
K_TRAILER = 5.0            # = K2 + K3   (K2 = 2.0)
K_TRUCK = -3.0             # = -K3       (K3 = 3.0)
K_D_ARTICULACION = 0.80
K_D_TRAILER = 0.60
# Extremo de la mezcla por severidad en modo pánico (más peso a la articulación, menos a la trayectoria)
K_TRAILER_PANICO = 4.1     # = K2p + K3p  (K2p = 0.5)
K_TRUCK_PANICO = -3.6      # = -K3p       (K3p = 3.6)
#
# Guía de ajuste (si en BeamNG ves oscilación o deriva):
#   - delta_F2 crece lentamente sin que el volante reaccione  -> subir K3 (más negativo K_TRUCK) y K_TRAILER en la misma cantidad
#   - el volante oscila rápido / "tiembla"                       -> bajar K3 o K_D_ARTICULACION
#   - el tráiler se aleja lentamente de la ruta                   -> subir K2 (subir K_TRAILER sin tocar K_TRUCK)
#   - snaking lento (periodo >20 s) alrededor de la ruta        -> subir K_D_TRAILER o alargar el lookahead
# Mantener siempre K_TRAILER > |K_TRUCK|  (K2 > 0)  y  K_TRUCK < 0  (K3 > 0).


def control_aditivo_histeresis(
        pos_trailer, psi_trailer, psi_truck, 
        delta_F2, delta_F2_rate, 
        trailer_roll, trailer_roll_rate, 
        velocidad, ruta, current_idx, 
        panic_mode: bool  = False,  # estado de la máquina de histéresis de pánico
        prev_e_trailer: float = 0.0,          # e_trailer crudo del tick anterior (para la derivada cruda)
        e_trailer_rate_filtrada: float = 0.0, # salida EMA anterior de la derivada de e_trailer (recursión del filtro)
        prev_seg_dir=None,  # (ux,uy) dirección unitaria del segmento de ruta usado el tick anterior; None en el primer tick
        dt: float         = 0.02,
        alpha_ema_trailer: float = 0.15,  # suavizado del EMA sobre e_trailer_rate (0=sin filtro, 1=total)
        min_lookahead=10.0, max_lookahead=20.0, k_lookahead_gain=3.5
        ):
    """
    Algoritmo de control aditivo con histéresis para marcha hacia atrás.
    Geometría consolidada a Pure Pursuit: el heading hacia un punto de
    lookahead dinámico (escalado por velocidad) resuelve tanto la posición
    lateral como el heading; ya no hay canal aditivo de cross-track (e_ct)
    separado. Implementa Prevención Derivativa de Plegamiento (sobre delta_F2,
    YA filtrada por EMA en main() antes de llegar aquí) y una segunda acción
    derivativa sobre la trayectoria (tasa de e_trailer, filtrada por EMA
    internamente), además de Gestión de Suspensión Activa (Atenuación Anti-Vuelco).

    Retorna: (steering, throttle, brake, next_idx, finished, panic_mode,
              e_trailer, e_trailer_rate_filtrada, seg_dir_usado, e_ct)
    panic_mode, e_trailer (como prev_e_trailer), e_trailer_rate_filtrada y
    seg_dir_usado (como prev_seg_dir) deben realimentarse en la siguiente iteración.
    e_ct es SOLO telemetría (error lateral para el CSV); no entra en el control.
    """
    if not len(ruta):
        return 0.0, 0.0, 1.0, current_idx, True, panic_mode, prev_e_trailer, e_trailer_rate_filtrada, prev_seg_dir, 0.0

    # Implementar Lookahead Dinámico
    # Al ir más rápido, busca puntos más lejanos limitando sobrecorrecciones.
    # Al ir más lento, acota el radio ganando enorme precisión.
    lookahead = np.clip(
        min_lookahead + (k_lookahead_gain * abs(velocidad)), 
        min_lookahead, 
        max_lookahead
    )

    # Encontrar el punto más cercano para al tráiler
    #
    # FILTRO DE CONTINUIDAD (anti-flip de índice): cerca de curvas o cuando el
    # camino pasa dos veces cerca del mismo punto en el espacio, el punto más
    # cercano "puro" puede saltar a un índice cuyo segmento apunta en una
    # dirección muy distinta al segmento usado el tick anterior -- lo que
    # corrompería tanto la base de la búsqueda de lookahead como la detección
    # de fin de ruta. Se buscan DOS candidatos en la misma pasada:
    #   - closest_idx_libre: el más cercano sin restricción (comportamiento
    #     original; sirve de fallback si ningún candidato es "continuo").
    #   - closest_idx_continuo: el más cercano cuyo segmento no se desvía más
    #     de 90° del segmento anterior (prev_seg_dir).
    min_dist = float('inf')
    closest_idx = current_idx
    min_dist_continuo = float('inf')
    closest_idx_continuo = None
    search_limit = min(current_idx + 100, len(ruta))

    for i in range(current_idx, search_limit):
        pt = ruta[i]
        dist = math.hypot(pos_trailer[0] - pt[0], pos_trailer[1] - pt[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

        if prev_seg_dir is not None and dist < min_dist_continuo:
            pt_sig = ruta[min(i + 1, len(ruta) - 1)]
            cand_dx = pt_sig[0] - pt[0]
            cand_dy = pt_sig[1] - pt[1]
            cand_len = math.hypot(cand_dx, cand_dy)
            if cand_len > 1e-6:
                cos_ang = (cand_dx / cand_len) * prev_seg_dir[0] + (cand_dy / cand_len) * prev_seg_dir[1]
                if cos_ang >= 0.0:  # <= 90 grados de diferencia respecto al segmento anterior
                    min_dist_continuo = dist
                    closest_idx_continuo = i

    # Si hay un candidato continuo, se prefiere sobre el más cercano "libre".
    # Si NINGÚN candidato pasó el filtro (ej. una curva cerrada legítima de
    # más de 90°), se cae de vuelta al resultado sin restricción para no
    # dejar al controlador congelado indefinidamente.
    if closest_idx_continuo is not None:
        closest_idx = closest_idx_continuo
        min_dist = min_dist_continuo

    # Dirección del segmento en closest_idx: sigue sirviendo como referencia
    # de continuidad para el PRÓXIMO tick. closest_idx sigue siendo la base
    # del avance de ruta y del punto de arranque de la búsqueda de lookahead,
    # así que la guarda de continuidad sigue siendo valiosa aunque e_ct (su
    # motivo original) ya no exista como término aditivo.
    #
    # e_ct (cross-track error): SOLO TELEMETRÍA. Distancia perpendicular con signo
    # del tráiler al segmento de ruta en closest_idx (positivo = a la izquierda del
    # sentido de avance de la ruta; misma convención que error_lateral en forward y
    # LQR). NO entra en la ley de control (Pure Pursuit consolidado): se devuelve
    # únicamente para registrar el error lateral real en el CSV de evaluación.
    e_ct = 0.0
    seg_dir_usado = prev_seg_dir
    if closest_idx < len(ruta) - 1:
        pt_a_idx = ruta[closest_idx]
        pt_b_idx = ruta[closest_idx + 1]
        seg_dx_idx = pt_b_idx[0] - pt_a_idx[0]
        seg_dy_idx = pt_b_idx[1] - pt_a_idx[1]
        seg_len_idx = math.hypot(seg_dx_idx, seg_dy_idx)
        if seg_len_idx > 1e-6:
            seg_dir_usado = (seg_dx_idx / seg_len_idx, seg_dy_idx / seg_len_idx)
            # Proyección de (tráiler - punto A) sobre la normal izquierda (-dy, dx)/|seg|
            e_ct = ((pos_trailer[0] - pt_a_idx[0]) * (-seg_dy_idx / seg_len_idx)
                    + (pos_trailer[1] - pt_a_idx[1]) * ( seg_dx_idx / seg_len_idx))

    # Busca el punto "lookahead" a una distancia de la posición actual del TRÁILER
    # Por defecto apuntamos al último punto de la ruta en caso de que estemos cerca del final
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

    # Si estamos en el tramo final y la ruta se acaba, proyectamos matemáticamente
    # el último vector para que el "lookahead" nunca se reduzca a 0, evitando el jackknife de fin de ruta.
    if not found_target and len(ruta) > 1:
        pt_last = ruta[-1]
        pt_prev = ruta[-2]
        dir_x = pt_last[0] - pt_prev[0]
        dir_y = pt_last[1] - pt_prev[1]
        norm = math.hypot(dir_x, dir_y)
        if norm > 0:
            dir_x /= norm
            dir_y /= norm
        # Extrapolamos el punto objetivo artificialmente hacia el horizonte manteniendo el lookahead real
        target_pt = (pos_trailer[0] + dir_x * lookahead, pos_trailer[1] + dir_y * lookahead)

    # Calcular el ángulo objetivo (Heading objetivo hacia el punto)
    dx = target_pt[0] - pos_trailer[0]
    dy = target_pt[1] - pos_trailer[1]
    yaw_objetivo = math.atan2(dy, dx)

    # MARCO DE REFERENCIA DEL HEADING (verificado contra truck_trailer.py):
    # psi_trailer/psi_truck = arctan2(state['dir']) = hacia dónde apunta la NARIZ
    # del vehículo, no hacia dónde se mueve. En el escenario 'bwd' la nariz mira
    # a -x y la ruta avanza hacia +x: en reversa recta psi ≈ ±pi mientras que
    # yaw_objetivo ≈ 0. Sin rotar el setpoint, un camión perfectamente alineado
    # da e = ∓pi y el volante salta a ±1 según de qué lado del corte ±pi caiga
    # psi (bang-bang puro). Con la rotación, e ≈ 0.
    # Diagnóstico rápido: la columna error_angular del CSV debe rondar 0; si
    # ronda ±3.14, poner OFFSET_HEADING_REVERSA = 0.0.
    yaw_objetivo_reversa = _envolver_angulo(yaw_objetivo + OFFSET_HEADING_REVERSA)

    # Errores direccionales individuales
    # Normalización de wrap angular con atan2(sin, cos): garantiza el camino
    # angular más corto en (-pi, pi] y es numéricamente más robusta que el
    # operador módulo con floats (evita acumulación de error de precisión
    # cerca de +-pi tras muchas iteraciones).
    e_trailer = _envolver_angulo(yaw_objetivo_reversa - psi_trailer)
    e_truck   = _envolver_angulo(yaw_objetivo_reversa - psi_truck)

    # Acción derivativa sobre la TRAYECTORIA (segunda D, distinta de la de
    # articulación): amortigua la velocidad de cambio de e_trailer para
    # frenar la oscilación respecto a la ruta antes de que cruce el setpoint.
    # EMA: filtrado_nuevo = alpha*filtrado_anterior + (1-alpha)*dato_crudo.
    #
    # CRÍTICO: e_trailer vive en (-pi, pi]. Si se resta crudo, al cruzar la
    # frontera +-pi la diferencia salta ~2*pi aunque el heading físico casi
    # no cambió ("derivative kick"). Envolver la diferencia ANTES de dividir
    # por dt evita esa alucinación de cientos de rad/s.
    e_trailer_rate_cruda = _envolver_angulo(e_trailer - prev_e_trailer) / dt
    e_trailer_rate_filtrada = (
        alpha_ema_trailer * e_trailer_rate_filtrada
        + (1 - alpha_ema_trailer) * e_trailer_rate_cruda
    )

    # CONSOLIDACIÓN A PURE PURSUIT: se elimina el canal aditivo de cross-track
    # (e_ct/integral_ct) que existía aquí. yaw_objetivo ya es el rumbo hacia
    # un punto de lookahead dinámico (escalado por velocidad) -- geometría de
    # Pure Pursuit pura. Si el tráiler está desplazado lateralmente del
    # camino, el rumbo a ese punto fijo-adelante deja de coincidir con el
    # heading local de la ruta; esa discrepancia YA es la corrección lateral,
    # sin necesidad de un canal e_ct aparte que usaba una referencia
    # geométrica distinta (el punto más cercano) y competía con esta señal.
    #
    # TRADE-OFF: a diferencia del canal anterior, Pure Pursuit puro no tiene
    # acción integral. Un sesgo físico constante (fricción asimétrica, etc.)
    # podría dejar un offset lateral pequeño pero persistente en tramos
    # rectos largos que esto ya no corrige. Si aparece, es preferible
    # reintroducirlo como un trim de heading, no resucitar e_ct.

    # Ganancias (ver el bloque de constantes K_* al inicio del archivo).
    k_trailer = K_TRAILER
    k_truck = K_TRUCK
    # Derivativa sobre la articulación: frena la velocidad de cierre/apertura de delta_F2.
    k_d_articulacion = K_D_ARTICULACION
    # Derivativa sobre la TRAYECTORIA: amortigua la velocidad de cambio de e_trailer.
    k_d_trailer = K_D_TRAILER

    # Histéresis Anti-Jackknife en casos de pánico crítico
    # Implementación de la Prevención Derivativa de Plegamiento (Rate of Change)
    # Límite duro estricto (0.45 rad, apróx 25°).
    # Límite blando (0.30 rad) combidado con una inercia angular peligrosa (> 0.20 rad/s)
    limite_tijera_duro = 0.42       # rad
    limite_tijera_blando = 0.28     # rad
    limite_derivativo = 0.18        # rad/s
    salida_angulo = 0.12            # rad
    salida_v_angular = 0.08         # rad/s
    
    if not panic_mode:
    # Si la articulación se cierra peligrosamente por geometría O se está cerrando demasiado rápido
        if abs(delta_F2) > limite_tijera_duro or (abs(delta_F2) > limite_tijera_blando and abs(delta_F2_rate) > limite_derivativo):
            panic_mode = True
    else:
        # Condición de salida
        if abs(delta_F2) < salida_angulo and abs(delta_F2_rate) < salida_v_angular:
            panic_mode = False
        
        # Si no sale
        else:
            severidad = np.clip(
                (abs(delta_F2) - 0.12) / (limite_tijera_duro - 0.12),
                0.0, 1.0
            )

            # El extremo "sin pánico" (severidad=0) coincide con las ganancias base
            # (K_TRAILER / K_TRUCK): sin saltos de ganancia al entrar en panic_mode.
            k_trailer = K_TRAILER * (1.0 - severidad) + K_TRAILER_PANICO * severidad
            k_truck   = K_TRUCK   * (1.0 - severidad) + K_TRUCK_PANICO   * severidad

    # Integración Dinámica de Suspensión (8-DOF Active Yaw/Roll Control)
    # Evaluamos la estabilidad gravitacional de las masas (Trailer Roll)
    # Un "roll" de 0.08 rad (~4.5°) ya es peligroso para un tráiler con carga.
    limite_roll = 0.08
    limite_roll_rate = 0.15 # rad/s de oscilación de la cabina
    
    # Atenuación predeterminada (1.0 significa sin efecto)
    atenuacion_dinamica = 1.0
    
    if abs(trailer_roll) > limite_roll or abs(trailer_roll_rate) > limite_roll_rate:
        # Penaliza exponencialmente basándose en el exceso de balanceo
        exceso = max(abs(trailer_roll) / limite_roll, abs(trailer_roll_rate) / limite_roll_rate)
        # Reducimos la agresividad del volante (hasta un 40% de su capacidad total) para estabilizar la inercia lateral.
        atenuacion_dinamica = np.clip(1.0 / exceso, 0.4, 1.0)
    
    # Realimentación NEGATIVA de los errores (ver bloque de constantes K_*).
    steering_raw = (
        - k_trailer * e_trailer
        - k_truck   * e_truck
        - k_d_articulacion * delta_F2_rate
        - k_d_trailer      * e_trailer_rate_filtrada
    ) * atenuacion_dinamica

    # Al inyectarlo en BeamNG, steering > 0 gira la llanta a la derecha. 
    steering = np.clip(steering_raw, -1.0, 1.0)

    # Control de velocidad (Magnitud absoluta de V1) y Anti-derrape
    velocidad_objetivo = 2.0 # m/s hacia atrás (al ser gear=-1, throttle produce movimiento trasero)
    
    # Si la atenuación está muy activa (mucho roll), forzamos una reducción severa de la velocidad objetivo
    # para mitigar el derrape o vuelco inminente.
    velocidad_objetivo *= atenuacion_dinamica

    error_vel = velocidad_objetivo - velocidad

    kp_vel = 0.4
    if error_vel > 0:
        throttle = np.clip(0.1 + kp_vel * error_vel, 0.0, 1.0)
        brake = 0.0
    else:
        throttle = 0.0
        brake = np.clip(-kp_vel * error_vel * 0.5, 0.0, 1.0)

    # Verificar si hemos llegado al final de la ruta
    finished = False
    if closest_idx >= len(ruta) - 2 and min_dist < 2.0:
        finished = True
        return 0.0, 0.0, 1.0, closest_idx, finished, panic_mode, e_trailer, e_trailer_rate_filtrada, seg_dir_usado, e_ct

    return steering, throttle, brake, closest_idx, finished, panic_mode, e_trailer, e_trailer_rate_filtrada, seg_dir_usado, e_ct


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

    # Variables de estado inicial para el subsistema derivativo del jackknifing y roll
    previous_delta_F2 = 0.0
    previous_trailer_roll = 0.0
    last_time = time.time()

    # Estado para la máquina de histéresis de pánico
    panic_mode_state  = False

    # Estado para el filtro EMA de delta_F2_rate (punto 1 de la ronda anterior:
    # la derivada cruda es muy sensible al ruido del tick rate).
    # NOTA DE CONVENCIÓN: filtrado_nuevo = alpha*filtrado_anterior + (1-alpha)*crudo.
    # alpha pondera el PASADO -> alpha MÁS BAJO = filtro MÁS reactivo (menos lag).
    # Bajado de 0.3 a 0.15 para reducir el retraso de fase de la acción derivativa.
    delta_F2_rate_filtrada = 0.0
    alpha_rate = 0.15

    # Estado para la acción derivativa sobre la trayectoria (punto 2 de la ronda anterior)
    prev_e_trailer_state = 0.0
    e_trailer_rate_filtrada_state = 0.0

    # Estado para el filtro de continuidad de closest_idx (evita saltos de
    # índice cerca de curvas/rutas que se cruzan en el espacio)
    prev_seg_dir_state = None  # None en el primer tick: sin restricción todavía

    # Telemetría CSV (entrada de evaluacion_resultados.py)
    inicializar_telemetria("telemetria_backward_recta.csv")

    start_time_log = time.time()
    while(True):
        # Llama a read_sensors en modo reversa = True para levantar la Reverse Cam
        sensores = truck_trailer.read_sensors(is_reverse=True)
        current_time = time.time()
        dt = max(current_time - last_time, 1e-4) # el mximo entre ambos valores

        velocidad = sensores["v1"]
        psi_camion = sensores["psi_truck"]
        psi_trailer = sensores["psi_trailer"]
        delta_F2 = sensores["delta_F2"]
        pos_trailer = sensores["trailer_pos"]
        trailer_roll = sensores["trailer_roll"]

        # Calcular la tasa de cambio del ángulo de articulación (delta_F2_rate)
        # y suavizarla con un EMA: la derivada cruda es muy sensible al ruido
        # del tick rate, y esa sensibilidad alimentaba directamente el bang-bang.
        delta_F2_rate_cruda = (delta_F2 - previous_delta_F2) / dt
        delta_F2_rate_filtrada = alpha_rate * delta_F2_rate_filtrada + (1 - alpha_rate) * delta_F2_rate_cruda
        previous_delta_F2 = delta_F2

        # Calcular la fuerza lateral de balanceo (tasa de roll)
        trailer_roll_rate = (trailer_roll - previous_trailer_roll) / dt
        previous_trailer_roll = trailer_roll

        last_time = current_time

        # Control Aditivo (Pure Pursuit consolidado)
        # Utilizamos la nueva implementación que calcula el lookahead dinámicamente en base
        # a la velocidad (v1), acotándolo automáticamente dentro de parámetros predefinidos.
        (steering, throttle, brake,
         current_idx, finished,
         panic_mode_state,             # realimentar estado de histéresis
         e_trailer,                    # error angular actual (se realimenta como prev_e_trailer, ver abajo)
         e_trailer_rate_filtrada_state,# realimentar salida EMA de la derivada de e_trailer
         prev_seg_dir_state,           # realimentar dirección del segmento (continuidad de closest_idx)
         e_ct                          # error lateral: SOLO telemetría, no entra en el control
        ) = control_aditivo_histeresis(
            pos_trailer, psi_trailer, psi_camion,
            delta_F2, delta_F2_rate_filtrada,
            trailer_roll, trailer_roll_rate,
            velocidad, route_points, current_idx,
            panic_mode  = panic_mode_state,
            prev_e_trailer = prev_e_trailer_state,
            e_trailer_rate_filtrada = e_trailer_rate_filtrada_state,
            prev_seg_dir = prev_seg_dir_state,
            alpha_ema_trailer = 0.15,
            dt          = dt
        )
        prev_e_trailer_state = e_trailer  # para la derivada del siguiente tick

        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)

        current_time = time.time()

        t_sim = current_time - start_time_log
        # Se omite el tick de fin de ruta: ahí el controlador devuelve un frenado
        # (steering=0), no una muestra de seguimiento, y falsearía S_smooth.
        if not finished:
            registrar_estado(
                t_sim,
                pos_trailer[0], pos_trailer[1],
                route_points[current_idx][0], route_points[current_idx][1],
                e_ct, e_trailer, delta_F2, steering
            )

        if abs(delta_F2) >= 1:
            print("Jackknife :c")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            generar_grafico_evaluacion(titulo='Evaluación Controlador Aditivo (Marcha Atrás) Jackknife')
            break


        # Si llegamos al final de la ruta generada (en reversa)
        if finished:
            print("Ruta de reversa completada. Frenando y devolviendo control manual (en Neutro)...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            break
        
    generar_grafico_evaluacion(titulo='Evaluación Controlador Aditivo (Marcha Atrás)')
    input('Presione enter cuando termine la simulación en reversa...')

    # truck_trailer.bng.close()

if __name__ == "__main__":
    main()