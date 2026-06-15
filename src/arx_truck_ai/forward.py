import math
from arx_truck_ai.debugging_sys import registrar_estado, generar_grafico_evaluacion
from arx_truck_ai import bng_open
from beamngpy import Vehicle
import numpy as np
import time

def control_proporcional(pos_actual, yaw_actual, truck_roll, truck_roll_rate, velocidad, ruta, current_idx, min_lookahead=5.0, max_lookahead=15.0, k_lookahead_gain=3.5):
    """
    Algoritmo de control de seguimiento de trayectoria básico usando Pure Pursuit / Proporcional.
    Implementa Lookahead Dinámico basado en la velocidad y Active Yaw/Roll Control limitando adherencia.
    Retorna (steering, throttle, brake, next_idx, finished).

    Cada idx es un índice en el arreglo de puntos de la ruta.
    """
    if not len(ruta):
        return 0.0, 0.0, 1.0, current_idx, True

    # Implementar Lookahead Dinámico
    # Ajustar inteligentemente dónde "mira" el camión hacia adelante basado en la velocidad actual
    lookahead = np.clip(min_lookahead + (k_lookahead_gain * abs(velocidad)), min_lookahead, max_lookahead)

    # Encuentra el punto más cercano o continua desde el índice actual
    min_dist = float('inf')
    closest_idx = current_idx
    # Buscar adelante un tramo en la ruta para no devolverse
    search_limit = min(current_idx + 100, len(ruta))
    for i in range(current_idx, search_limit):
        pt = ruta[i]
        dist = math.hypot(pos_actual[0] - pt[0], pos_actual[1] - pt[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    # Busca el punto "lookahead" a una distancia de la posición actual
    target_idx = closest_idx
    for i in range(closest_idx, len(ruta)):
        pt = ruta[i]
        dist = math.hypot(pos_actual[0] - pt[0], pos_actual[1] - pt[1])
        if dist >= lookahead:
            target_idx = i
            break

    target_pt = ruta[target_idx]

    # Cálculo del Error Lateral (cross-track error)
    pt_actual_tr = ruta[closest_idx]
    pt_siguiente_tr = ruta[min(closest_idx + 1, len(ruta) - 1)]
    dx_path = pt_siguiente_tr[0] - pt_actual_tr[0]
    dy_path = pt_siguiente_tr[1] - pt_actual_tr[1]
    yaw_path = math.atan2(dy_path, dx_path)
    dx_tr = pos_actual[0] - pt_actual_tr[0]
    dy_tr = pos_actual[1] - pt_actual_tr[1]
    error_lateral = dy_tr * math.cos(yaw_path) - dx_tr * math.sin(yaw_path)

    # Calcular el error de orientación (heading error)
    dx = target_pt[0] - pos_actual[0]
    dy = target_pt[1] - pos_actual[1]
    yaw_objetivo = math.atan2(dy, dx)

    raw_error = yaw_objetivo - yaw_actual
    # Normaliza el error a [-pi, pi]
    yaw_error = (raw_error + math.pi) % (2 * math.pi) - math.pi

    # Integración Dinámica de Suspensión (8-DOF Active Yaw/Roll Control)
    # Evaluamos la estabilidad gravitacional de las masas (Truck Roll)
    limite_roll = 0.08
    limite_roll_rate = 0.15 
    
    # Atenuación predeterminada (1.0 significa sin efecto)
    atenuacion_dinamica = 1.0
    
    if abs(truck_roll) > limite_roll or abs(truck_roll_rate) > limite_roll_rate:
        # Penaliza exponencialmente basándose en el exceso de balanceo
        exceso = max(abs(truck_roll) / limite_roll, abs(truck_roll_rate) / limite_roll_rate)
        # Reducimos la agresividad del volante (hasta un 40% de su capacidad total) para evitar volcar.
        atenuacion_dinamica = np.clip(1.0 / exceso, 0.4, 1.0)

    # Ganancia proporcional para la dirección
    kp_steer = 1.2
    # El simulador toma valores positivos de volante (steering > 0) para girar a la derecha. 
    # Un "yaw_error" positivo significa que el objetivo está a la izquierda (anti-horario),
    # por lo tanto, debemos invertir el signo del volante para girar correctamente.
    steering = -kp_steer * yaw_error * atenuacion_dinamica
    # La máxima dirección de BeamNG entra en un rango de [-1, 1]
    steering = np.clip(steering, -1.0, 1.0)

    # Control de velocidad proporcional
    velocidad_objetivo = 6.0 # m/s
    
    # Si la atenuación está muy activa (mucho roll), forzamos una reducción de la velocidad objetivo
    # como un sistema de control de estabilidad activo (ESC) para devolver tracción al asfalto.
    velocidad_objetivo *= atenuacion_dinamica
    
    kp_vel = 0.5
    error_vel = velocidad_objetivo - velocidad

    throttle = 0.0
    brake = 0.0

    if error_vel > 0:
        throttle = np.clip(0.1 + kp_vel * error_vel, 0.0, 1.0)
    else:
        brake = np.clip(-kp_vel * error_vel * 0.5, 0.0, 1.0)

    # Verificar si hemos llegado al final de la ruta
    finished = False
    if closest_idx == len(ruta) - 1 and min_dist < 2.0:
        finished = True
        return 0.0, 0.0, 1.0, closest_idx, finished

    return steering, throttle, brake, closest_idx, finished, error_lateral, yaw_error

def main():
    start_time_log = time.time()
    truck_trailer, orig = bng_open.main()

    # Posición de origen para el vehículo

    truck_trailer.bng.control.pause()
    truck_trailer.bng.settings.set_deterministic(60)

    truck_trailer.bng.control.resume()

    current_idx = 0
    route_points = truck_trailer.route

    # Variables de estado inicial para el control yaw/roll dinámico
    previous_truck_roll = 0.0
    last_time = time.time()

    while(True):
        sensores = truck_trailer.read_sensors()

        current_time = time.time()
        dt = current_time - last_time
        if dt < 1e-4: 
            dt = 1e-4

        pos_camion = sensores["truck_pos"]
        yaw_camion = sensores["psi_truck"]
        v1 = sensores["v1"]
        truck_roll = sensores["truck_roll"]

        # Leemos delta_F2 (articulación) aunque estemos yendo hacia adelante, solo para la gráfica
        delta_F2 = sensores.get("delta_F2", 0.0)

        # Calcular la rotación/inercia de balanceo (tasa de roll)
        truck_roll_rate = (truck_roll - previous_truck_roll) / dt
        previous_truck_roll = truck_roll
        last_time = current_time

        # Calcula el control proporcional simple para continuar por la ruta generada
        # Utiliza la nueva configuración de lookahead dinámico y control amortiguado (Active Yaw)
        steering, throttle, brake, current_idx, finished, error_lateral, yaw_error = control_proporcional(
            pos_camion, yaw_camion, truck_roll, truck_roll_rate, v1, route_points, current_idx
        )

        # Aplicar el comando al vehículo en el simulador
        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0)

        t_sim = current_time - start_time_log
        registrar_estado(t_sim, error_lateral, yaw_error, delta_F2, steering)

        # Si llegamos al final de la trayectoria, detenemos el control autónomo
        if finished:
            print("Ruta completada. Frenando y devolviendo control manual...")
            # Se resetean los controles para dejar al usuario al mando
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0)
            break
        
    generar_grafico_evaluacion(titulo='Evaluación Controlador Proporcional (Marcha Adelante)')
    input('Presione enter cuando termine la simulación...')

    # Disconnect BeamNG
    #truck_trailer.bng.disconnect()

    # Or close the simulator
    truck_trailer.bng.close()

if __name__ == "__main__":
    main()