import math
from arx_truck_ai import bng_open
from arx_truck_ai.debugging_sys import registrar_estado, generar_grafico_evaluacion
from beamngpy import Vehicle
import numpy as np
import time

def control_aditivo_histeresis(
        pos_trailer, psi_trailer, psi_truck, 
        delta_F2, delta_F2_rate, 
        trailer_roll, trailer_roll_rate, 
        velocidad, ruta, current_idx, 
        integral_ct: float = 0.0,  # integral del error de carril (anti-sesgo)
        panic_mode: bool  = False,  # estado de la máquina de histéresis de pánico
        dt: float         = 0.02,
        min_lookahead=6.0, max_lookahead=15.0, k_lookahead_gain=3.5
        ):
    """
    Algoritmo de control aditivo con histéresis para marcha hacia atrás.
    Implementa Lookahead Dinámico basado en la velocidad, Prevención Derivativa de Plegamiento
    y Gestión de Suspensión Activa (Atenuación Anti-Vuelco).

    Retorna: (steering, throttle, brake, next_idx, finished, e_trailer, panic_mode)
    Los dos últimos valores deben realimentarse en la siguiente iteración.
    """
    if not len(ruta):
        return 0.0, 0.0, 1.0, current_idx, True, integral_ct, panic_mode

    # Implementar Lookahead Dinámico
    # Al ir más rápido, busca puntos más lejanos limitando sobrecorrecciones.
    # Al ir más lento, acota el radio ganando enorme precisión.
    lookahead = np.clip(
        min_lookahead + (k_lookahead_gain * abs(velocidad)), 
        min_lookahead, 
        max_lookahead
    )

    # Encontrar el punto más cercano para al tráiler
    min_dist = float('inf')
    closest_idx = current_idx
    search_limit = min(current_idx + 100, len(ruta))

    for i in range(current_idx, search_limit):
        pt = ruta[i]
        dist = math.hypot(pos_trailer[0] - pt[0], pos_trailer[1] - pt[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

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

    # Dado que vamos en reversa, la parte trasera del tráiler es la que debe apuntar al objetivo. 
    # Físicamente, el yaw mide hacia dónde apunta la nariz del vehículo. Por tanto, el "yaw objetivo de reversa"
    # es el opuesto al yaw frontal.
    yaw_objetivo_reversa = (yaw_objetivo + math.pi) % (2 * math.pi) - math.pi

    # Errores direccionales individuales
    e_trailer = (yaw_objetivo_reversa - psi_trailer + math.pi) % (2 * math.pi) - math.pi
    e_truck   = (yaw_objetivo_reversa - psi_truck   + math.pi) % (2 * math.pi) - math.pi

    # Los errores angulares (e_trailer, e_truck) controlan la ORIENTACIÓN
    # del conjunto. El cross-track controla la POSICIÓN lateral: cuánto
    # se ha desplazado el tráiler perpendicularmente al camino.
    #
    # Un controlador que solo usa errores angulares puede seguir la dirección
    # correcta pero desplazado lateralmente (como un corredor que corre
    # paralelo a la línea pero no sobre ella). El cross-track cierra ese loop.
    #
    # Signo: positivo = tráiler a la IZQUIERDA del segmento (en el sentido de marcha).
    # NOTA: Si tras las pruebas el cross-track corrige en sentido contrario,
    #       invertir el signo de k_ct (de +0.18 a -0.18).
    e_ct = 0.0
    if closest_idx < len(ruta) - 1:
        pt_a    = ruta[closest_idx]
        pt_b    = ruta[closest_idx + 1]
        seg_dx  = pt_b[0] - pt_a[0]
        seg_dy  = pt_b[1] - pt_a[1]
        seg_len = math.hypot(seg_dx, seg_dy)

        if seg_len > 0.01:  # Evitar división por cero en segmentos degenerados
            # Normal unitaria al segmento (apunta 90° a la izquierda del sentido de marcha)
            nx = -seg_dy / seg_len
            ny =  seg_dx / seg_len
            # Vector del punto A de la ruta al tráiler
            to_t_x = pos_trailer[0] - pt_a[0]
            to_t_y = pos_trailer[1] - pt_a[1]
            # Proyección sobre la normal = distancia perpendicular con signo
            e_ct = to_t_x * nx + to_t_y * ny

    k_i_ct          = 0.025   # Ganancia integral muy conservadora; ajustar a 0 si no hay sesgo
    integral_ct_max = 0.20    # Anti-windup: límite en unidades de steering [-1, 1]

    if abs(e_ct) < 1.0 and not panic_mode:
        # Zona de seguimiento fino: acumula el sesgo lentamente
        integral_ct += e_ct * dt
        integral_ct  = np.clip(integral_ct, -integral_ct_max, integral_ct_max)
    elif abs(e_ct) > 2.0:
        # Corrección grande en curso: la integral acumulada ya no es válida;
        # descarga gradual (10% por ciclo) en lugar de reset brusco
        integral_ct *= 0.90

    # Ganancias del Contramovimiento
    # En BeamNG, steering > 0 gira las ruedas a la derecha, lo que empuja la cola del 
    # camión a la izquierda, la nariz del tráiler a la izquierda, y su cola a la derecha.
    # Por ende, necesitamos k_trailer POSITIVO frente al error del tráiler.
    k_trailer = 0.70  
    k_truck = -0.80
    k_ct = 0.18

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

            k_trailer = 0.70 * (1.0 - severidad) + 0.12 * severidad
            k_truck   = -0.80 * (1.0 - severidad) + (-1.10) * severidad
            k_ct      = 0.18  * (1.0 - severidad) + 0.04  * severidad

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
    
    steering_raw = (
          k_trailer  * e_trailer
        + k_truck    * e_truck
        + k_ct       * e_ct
        + k_i_ct    * integral_ct
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
        return 0.0, 0.0, 1.0, closest_idx, finished, integral_ct, panic_mode

    return steering, throttle, brake, closest_idx, finished, integral_ct, panic_mode


def main():
    start_time_log = time.time()
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

    # Estados para el control integral
    integral_ct_state = 0.0    # integral del error de carril
    panic_mode_state  = False  # estado de la histéresis de pánico

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
        delta_F2_rate = (delta_F2 - previous_delta_F2) / dt
        previous_delta_F2 = delta_F2

        # Calcular la fuerza lateral de balanceo (tasa de roll)
        trailer_roll_rate = (trailer_roll - previous_trailer_roll) / dt
        previous_trailer_roll = trailer_roll

        last_time = current_time

        # Control Aditivo 
        # Utilizamos la nueva implementación que calcula el lookahead dinámicamente en base
        # a la velocidad (v1), acotándolo automáticamente dentro de parámetros predefinidos.
        (steering, throttle, brake,
         current_idx, finished,
         integral_ct_state,   # realimentar integral acumulada
         panic_mode_state      # realimentar estado de histéresis
        ) = control_aditivo_histeresis(
            pos_trailer, psi_trailer, psi_camion,
            delta_F2, delta_F2_rate,
            trailer_roll, trailer_roll_rate,
            velocidad, route_points, current_idx,
            integral_ct = integral_ct_state,
            panic_mode  = panic_mode_state,
            dt          = dt
        )

        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)

        current_time = time.time()

        t_sim = current_time - start_time_log
        registrar_estado(t_sim, psi_trailer, psi_camion, delta_F2, steering)

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
