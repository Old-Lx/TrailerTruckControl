import math
from arx_truck_ai import bng_open
from arx_truck_ai import fin_ruta as fr
from arx_truck_ai.debugging_sys import (registrar_muestra, generar_grafico_evaluacion,
                                       inicializar_telemetria, finalizar_telemetria)
from arx_truck_ai import truck_trailer as tt
from arx_truck_ai.fin_ruta import (distancia_restante, velocidad_limite_fin, debe_frenar,
                                   punto_objetivo_extendido, indice_segmento, detener_en_fin, ControlVelocidad)
import numpy as np

# ---------------------------------------------------------------------------
# Fin de ruta (ver fin_ruta.py). El punto que debe detenerse en el final es trailer_pos.
V_CRUCERO = 2.0          # m/s   velocidad de crucero en reversa
A_APROXIMACION = 0.3     # m/s²  desaceleración del perfil de llegada: v_obj = sqrt(2*a*d_restante)
V_LLEGADA = 0.5          # m/s   velocidad mínima del perfil mientras quede ruta
A_FRENADO = 1.5          # m/s²  desaceleración que se ASUME con freno a fondo (conservadora)
MARGEN_PARADA = 0.3      # m     margen extra antes del punto final
KP_VELOCIDAD = 0.4       # ganancia proporcional del control de velocidad (fin_ruta.ControlVelocidad)
# ---------------------------------------------------------------------------


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
        prev_e_trailer=None,                  # e_trailer crudo del tick anterior (None en el primer tick: derivada 0)
        e_trailer_rate_filtrada: float = 0.0, # salida EMA anterior de la derivada de e_trailer (recursión del filtro)
        prev_seg_dir=None,  # (ux,uy) dirección unitaria del segmento de ruta usado el tick anterior; None en el primer tick
        dt: float         = 0.02,
        alpha_ema_trailer: float = 0.15,  # suavizado del EMA sobre e_trailer_rate (0=sin filtro, 1=total)
        min_lookahead=10.0, max_lookahead=20.0, k_lookahead_gain=3.5,
        ctrl_vel=None,                    # fin_ruta.ControlVelocidad de la corrida (None: ley 'original', sin estado)
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
    # En el último punto se usa el último segmento (antes quedaba e_ct = 0 en el tramo final).
    e_ct = 0.0
    seg_dir_usado = prev_seg_dir
    idx_seg = indice_segmento(ruta, closest_idx)
    if len(ruta) > 1:
        pt_a_idx = ruta[idx_seg]
        pt_b_idx = ruta[idx_seg + 1]
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

    # Si estamos en el tramo final y la ruta se acaba, el objetivo se toma sobre la prolongación
    # recta del último segmento ANCLADA EN EL ÚLTIMO PUNTO, a distancia `lookahead` del tráiler.
    # El lookahead nunca se reduce a 0 (evita el jackknife de fin de ruta) y el tráiler sigue siendo
    # atraído lateralmente hacia el punto final. (Proyectarlo desde el propio tráiler solo alineaba
    # el rumbo: el conjunto seguía paralelo a la ruta, desviado, y se alejaba del punto final.)
    if not found_target and len(ruta) > 1:
        target_pt = punto_objetivo_extendido(ruta, pos_trailer, lookahead)

    # Distancia que falta hasta el final, medida A LO LARGO de la ruta (negativa si ya se pasó).
    d_restante = distancia_restante(ruta, closest_idx, pos_trailer)

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
    e_trailer_rate_cruda = 0.0 if prev_e_trailer is None else _envolver_angulo(e_trailer - prev_e_trailer) / dt
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

    # Atenuación del volante por balanceo (roll) de la suspensión
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

    # Control de velocidad (Magnitud absoluta de V1) y Anti-derrape.
    # Al ser gear=-1, throttle produce movimiento hacia atrás. Perfil de llegada: cerca del final
    # la velocidad objetivo baja como sqrt(2*a*d_restante) para llegar despacio al punto final.
    velocidad_objetivo = min(V_CRUCERO, velocidad_limite_fin(d_restante, A_APROXIMACION, V_LLEGADA))
    
    # Si la atenuación está muy activa (mucho roll), forzamos una reducción severa de la velocidad objetivo
    # para mitigar el derrape o vuelco inminente.
    velocidad_objetivo *= atenuacion_dinamica

    # Control de velocidad común (fin_ruta.ControlVelocidad: PI o la ley 'original')
    if ctrl_vel is None:
        ctrl_vel = ControlVelocidad(KP_VELOCIDAD, V_CRUCERO, modo='original')
    throttle, brake = ctrl_vel(velocidad_objetivo, velocidad, dt)

    # Verificar si hemos llegado al final de la ruta: frenar a fondo cuando la distancia de parada
    # alcanza a la que queda (o si ya se pasó). Se mide a lo largo de la ruta, así que funciona
    # aunque el tráiler llegue desviado lateralmente (antes exigía estar a < 2 m del último punto).
    finished = False
    if debe_frenar(d_restante, velocidad, A_FRENADO, MARGEN_PARADA):
        finished = True
        return steering, 0.0, 1.0, closest_idx, finished, panic_mode, e_trailer, e_trailer_rate_filtrada, seg_dir_usado, e_ct

    return steering, throttle, brake, closest_idx, finished, panic_mode, e_trailer, e_trailer_rate_filtrada, seg_dir_usado, e_ct


def main():
    direction = 'bwd'
    truck_trailer, orig = bng_open.main(direction)

    # Al poner la caja de cambios del camión en Reversa (Gear = -1), el acelerador lo desplazará hacia atrás
    truck_trailer.iniciar_simulacion(gear=-1)

    current_idx = 0
    route_points = truck_trailer.route
    nombre_ruta = truck_trailer.nombre_ruta

    # Telemetría (entrada de evaluacion_resultados.py): una fila por paso de simulación
    inicializar_telemetria(f"telemetria_backward_{nombre_ruta}.csv", puntos_ruta=route_points, sentido=-1,
                           punto_control='trailer', algoritmo='backward', ruta=nombre_ruta,
                           tipo_ruta=tt.TIPO_RUTA, modo='sincrono' if tt.MODO_SINCRONO else 'libre',
                           frecuencia_simulacion=tt.FRECUENCIA_SIMULACION, pasos_por_segundo=tt.PASOS_POR_SEGUNDO,
                           frecuencia_control=tt.FRECUENCIA_SIMULACION / truck_trailer.decimacion_control
                           if tt.MODO_SINCRONO else None,
                           control_velocidad=fr.MODO_CONTROL_VELOCIDAD)

    # Variables de estado inicial para el subsistema derivativo del jackknifing y roll.
    # None: en el primer ciclo de control las derivadas valen 0 (antes se derivaba contra 0 y, si el
    # estado inicial no era nulo, el primer comando de volante tenía un pico falso).
    previous_delta_F2 = None
    previous_trailer_roll = None

    # Estado para la máquina de histéresis de pánico
    panic_mode_state  = False

    # Estado para el filtro EMA de delta_F2_rate (la derivada cruda es muy sensible al ruido).
    # NOTA DE CONVENCIÓN: filtrado_nuevo = alpha*filtrado_anterior + (1-alpha)*crudo.
    # alpha pondera el PASADO -> alpha MÁS BAJO = filtro MÁS reactivo (menos lag).
    delta_F2_rate_filtrada = 0.0
    alpha_rate = 0.15

    # Estado para la acción derivativa sobre la trayectoria
    prev_e_trailer_state = None   # None en el primer ciclo (ver arriba)
    e_trailer_rate_filtrada_state = 0.0

    # Estado para el filtro de continuidad de closest_idx (evita saltos de
    # índice cerca de curvas/rutas que se cruzan en el espacio)
    prev_seg_dir_state = None  # None en el primer tick: sin restricción todavía

    t0 = None
    t_ultimo_control = None
    steering, throttle, brake = 0.0, 0.0, 0.0
    ctrl_vel = ControlVelocidad(KP_VELOCIDAD, V_CRUCERO)
    e_trailer = e_ct = None
    finished = False
    resultado, info_fin = 'completada', {}
    ciclo = 0

    while True:
        # En modo síncrono el control se recalcula cada `decimacion_control` pasos y la telemetría se
        # registra en todos; en modo libre ambos ocurren en cada iteración.
        es_ciclo_control = (ciclo % truck_trailer.decimacion_control == 0)
        # Llama a read_sensors en modo reversa = True para levantar la Reverse Cam
        sensores = truck_trailer.read_sensors(is_reverse=True, completo=es_ciclo_control)
        t = sensores['t_sim']
        if t0 is None:
            t0 = t
        t_rel = t - t0

        velocidad = sensores["v1"]
        psi_camion = sensores["psi_truck"]
        psi_trailer = sensores["psi_trailer"]
        delta_F2 = sensores["delta_F2"]
        pos_trailer = sensores["trailer_pos"]
        trailer_roll = sensores["trailer_roll"]

        if es_ciclo_control:
            dt = (t - t_ultimo_control) if t_ultimo_control is not None else (truck_trailer.dt_control or 0.1)
            dt = max(dt, 1e-4)
            t_ultimo_control = t

            # Tasa de cambio del ángulo de articulación (delta_F2_rate), suavizada con un EMA
            delta_F2_rate_cruda = 0.0 if previous_delta_F2 is None else (delta_F2 - previous_delta_F2) / dt
            delta_F2_rate_filtrada = alpha_rate * delta_F2_rate_filtrada + (1 - alpha_rate) * delta_F2_rate_cruda
            previous_delta_F2 = delta_F2

            # Calcular la fuerza lateral de balanceo (tasa de roll)
            trailer_roll_rate = 0.0 if previous_trailer_roll is None else (trailer_roll - previous_trailer_roll) / dt
            previous_trailer_roll = trailer_roll

            # Control Aditivo (Pure Pursuit consolidado)
            (steering, throttle, brake,
             current_idx, finished,
             panic_mode_state,             # realimentar estado de histéresis
             e_trailer,                    # error angular actual (se realimenta como prev_e_trailer, ver abajo)
             e_trailer_rate_filtrada_state,# realimentar salida EMA de la derivada de e_trailer
             prev_seg_dir_state,           # realimentar dirección del segmento (continuidad de closest_idx)
             e_ct                          # error lateral del controlador (diagnóstico)
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
                dt          = dt,
                ctrl_vel    = ctrl_vel
            )
            prev_e_trailer_state = e_trailer  # para la derivada del siguiente ciclo de control

            truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)

        # Telemetría: el tick de fin de ruta es un comando de frenado, no una muestra de seguimiento
        if not finished:
            registrar_muestra(t_rel, sensores, steering, control_actualizado=es_ciclo_control,
                              error_lateral_ctrl=e_ct, error_angular_ctrl=e_trailer,
                              throttle=throttle, brake=brake)

        if abs(delta_F2) >= 1:
            print("Jackknife :c")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            resultado, info_fin = 'jackknife', dict(delta_F2_final_rad=float(delta_F2))
            break

        # Si llegamos al final de la ruta generada (en reversa)
        if finished:
            print("Fin de ruta: frenando hasta detener el conjunto...")
            # Frena hasta detenerse y deja el camión con freno de mano, sin freno de servicio y en neutro.
            sobrepaso, d_fin = detener_en_fin(truck_trailer, route_points, current_idx, 'trailer_pos',
                                              steering=steering, is_reverse=True, gear=-1)
            print(f"Conjunto detenido. Sobrepaso del tráiler a lo largo de la ruta: {sobrepaso:+.2f} m "
                  f"(+ = se pasó del punto final) | distancia al punto final: {d_fin:.2f} m")
            resultado, info_fin = 'completada', dict(sobrepaso_m=sobrepaso, distancia_final_m=d_fin)
            break
        if t_rel > tt.DURACION_MAXIMA_S:
            print(f"Tiempo máximo del experimento ({tt.DURACION_MAXIMA_S:.0f} s) alcanzado: deteniendo...")
            sobrepaso, d_fin = detener_en_fin(truck_trailer, route_points, current_idx, 'trailer_pos',
                                              steering=steering, is_reverse=True, gear=-1)
            resultado, info_fin = 'tiempo_maximo', dict(sobrepaso_m=sobrepaso, distancia_final_m=d_fin)
            break

        truck_trailer.avanzar()      # modo síncrono: un paso de simulación (1/60 s); modo libre: nada
        ciclo += 1

    finalizar_telemetria(resultado, **info_fin)
    truck_trailer.liberar_simulacion()
    titulo = 'Evaluación Controlador Aditivo (Marcha Atrás)' + (' Jackknife' if resultado == 'jackknife' else '')
    generar_grafico_evaluacion(titulo=titulo)
    input('Presione enter cuando termine la simulación en reversa...')

    # truck_trailer.bng.close()

if __name__ == "__main__":
    main()
