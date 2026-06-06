import math
from arx_truck_ai import bng_open
from arx_truck_ai.debugging_sys import registrar_estado
from beamngpy import Vehicle
import numpy as np
import time

def control_aditivo_histeresis(pos_trailer, psi_trailer, psi_truck, delta_F2, delta_F2_rate, trailer_roll, trailer_roll_rate, velocidad, ruta, current_idx, min_lookahead=5.0, max_lookahead=15.0, k_lookahead_gain=3.5):
    """
    Algoritmo de control aditivo con histéresis para marcha hacia atrás.
    Implementa Lookahead Dinámico basado en la velocidad, Prevención Derivativa de Plegamiento
    y Gestión de Suspensión Activa (Atenuación Anti-Vuelco).
    Retorna (steering, throttle, brake, next_idx, finished).
    """
    if len(ruta) == 0:
        return 0.0, 0.0, 1.0, current_idx, True

    # Implementar Lookahead Dinámico
    # Al ir más rápido, busca puntos más lejanos limitando sobrecorrecciones.
    # Al ir más lento, acota el radio ganando enorme precisión.
    lookahead = np.clip(min_lookahead + (k_lookahead_gain * abs(velocidad)), min_lookahead, max_lookahead)

    # Encontrar el punto más cercano para el TRÁILER
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
    e_truck = (yaw_objetivo_reversa - psi_truck + math.pi) % (2 * math.pi) - math.pi

    # Ganancias del Contramovimiento
    # En BeamNG, steering > 0 gira las ruedas a la derecha, lo que empuja la cola del 
    # camión a la izquierda, la nariz del tráiler a la izquierda, y su cola a la derecha.
    # Por ende, necesitamos k_trailer POSITIVO frente al error del tráiler.
    #
    # Para eliminar el "S-shape" (serpenteo/snaking), necesitamos un control altamente amortiguado.
    # Si reescribimos la ecuación aditiva: steering = K_path * e_trailer + K_damp * delta_F2
    # El S-shape ocurre porque el camión se articula más rápido de lo que corrige el camino.
    # Ajustamos a: Path gain (suave) = 0.25, Damping gain (agresivo) = 0.85
    # Despejando: k_truck = -0.85, k_trailer = 0.25 - k_truck = 1.10
    k_trailer = 1.10  
    k_truck = -0.85      

    # Histéresis Anti-Jackknife en casos de pánico crítico
    # Implementación de la Prevención Derivativa de Plegamiento (Rate of Change)
    # Límite duro estricto (0.45 rad, apróx 25°).
    # Límite blando (0.30 rad) combidado con una inercia angular peligrosa (> 0.20 rad/s)
    limite_tijera_duro = 0.45
    limite_tijera_blando = 0.30
    urgencia_derivativa = 0.20
    
    # Si la articulación se cierra peligrosamente por geometría O se está cerrando demasiado rápido
    if abs(delta_F2) > limite_tijera_duro or (abs(delta_F2) > limite_tijera_blando and abs(delta_F2_rate) > urgencia_derivativa):
        # Prioridad ABSOLUTA a enderezar el camión bajo el tráiler atenuando la búsqueda espacial
        k_trailer = 0.15 
        k_truck = -1.20

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
    
    steering_raw = (k_trailer * e_trailer + k_truck * e_truck) * atenuacion_dinamica
    
    # Del mismo modo que el control frontal asume negativo para anti-horario, puede requerirse ajuste empírico en reversa.
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
        return 0.0, 0.0, 1.0, closest_idx, finished

    return steering, throttle, brake, closest_idx, finished


def main():
    start_time_log = time.time()
    truck_trailer, orig = bng_open.main()

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

    while(True):
        # Llama a read_sensors en modo reversa = True para levantar la Reverse Cam
        sensores = truck_trailer.read_sensors(is_reverse=True)

        current_time = time.time()
        dt = current_time - last_time
        if dt < 1e-4: 
            dt = 1e-4  # Límite anti-fallo para división entre 0 por extrema rapidez de clock

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
        steering, throttle, brake, current_idx, finished = control_aditivo_histeresis(
            pos_trailer, psi_trailer, psi_camion, delta_F2, delta_F2_rate, trailer_roll, trailer_roll_rate, velocidad, route_points, current_idx
        )

        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)

        current_time = time.time()

        t_sim = current_time - start_time_log
        registrar_estado(t_sim, psi_trailer, psi_camion, delta_F2, steering)

        # Si llegamos al final de la ruta generada (en reversa)
        if finished:
            print("Ruta de reversa completada. Frenando y devolviendo control manual (en Neutro)...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            break

    input('Presione enter cuando termine la simulación en reversa...')

    # truck_trailer.bng.close()

if __name__ == "__main__":
    main()
