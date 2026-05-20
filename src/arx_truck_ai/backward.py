import math
from arx_truck_ai import bng_open
from beamngpy import Vehicle
import numpy as np
import time

def control_aditivo_histeresis(pos_trailer, psi_trailer, psi_truck, delta_F2, velocidad, ruta, current_idx, lookahead=6.0):
    """
    Algoritmo de control aditivo con histéresis para marcha hacia atrás.
    Retorna (steering, throttle, brake, next_idx, finished).
    """
    if len(ruta) == 0:
        return 0.0, 0.0, 1.0, current_idx, True

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

    # 2. Busca el punto "lookahead" a una distancia de la posición actual del TRÁILER
    target_idx = closest_idx
    for i in range(closest_idx, len(ruta)):
        pt = ruta[i]
        dist = math.hypot(pos_trailer[0] - pt[0], pos_trailer[1] - pt[1])
        if dist >= lookahead:
            target_idx = i
            break

    target_pt = ruta[target_idx]

    # 3. Calcular el ángulo objetivo (Heading objetivo hacia el punto)
    dx = target_pt[0] - pos_trailer[0]
    dy = target_pt[1] - pos_trailer[1]
    yaw_objetivo = math.atan2(dy, dx)

    # Dado que vamos en reversa, la parte trasera del tráiler es la que debe apuntar al objetivo. 
    # Físicamente, el yaw mide hacia dónde apunta la nariz del vehículo. Por tanto, el "yaw objetivo de reversa"
    # es el opuesto al yaw frontal.
    yaw_objetivo_reversa = (yaw_objetivo + math.pi) % (2 * math.pi) - math.pi

    # 4. Errores direccionales individuales
    e_trailer = (yaw_objetivo_reversa - psi_trailer + math.pi) % (2 * math.pi) - math.pi
    e_truck = (yaw_objetivo_reversa - psi_truck + math.pi) % (2 * math.pi) - math.pi

    # 5. Ganancias Proporcionales base
    # Para el tráiler, en reversa, si el error es positivo (izquierda), el camión debe girar el volante a la derecha (steering positivo)
    # para enviar el remolque a la izquierda.
    k_trailer = 1.5   # Tráiler empuja
    k_truck = -0.5    # El camión sigue la misma línea general para evitar tijera

    # 6. Histéresis (Anti-Jackknife)
    # Si la articulación se cierra peligrosamente (ej. > 25° aprox 0.43 rad)
    limite_tijera = 0.45
    if abs(delta_F2) > limite_tijera:
        # Se penaliza el seguimiento del tráiler, y se prioriza que el camión se alinee con el tráiler
        k_trailer = 0.8
        k_truck = -1.2
        # print(f"Alerta de tijera (Articulación: {delta_F2:.2f} rad). Reduciendo giro...")

    steering_raw = k_trailer * e_trailer + k_truck * e_truck
    
    # Del mismo modo que el control frontal asume negativo para anti-horario, puede requerirse ajuste empírico en reversa.
    # Al inyectarlo en BeamNG, steering > 0 gira la llanta a la derecha. 
    steering = np.clip(steering_raw, -1.0, 1.0)

    # 7. Control de velocidad (Magnitud absoluta de V1)
    velocidad_objetivo = 2.0 # m/s hacia atrás (al ser gear=-1, throttle produce movimiento trasero)
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
    truck_trailer, orig = bng_open.main()

    truck_trailer.bng.control.pause()
    truck_trailer.bng.settings.set_deterministic(60)

    # Al poner la caja de cambios del camión en Reversa (Gear = -1), el acelerador lo desplazará hacia atrás
    truck_trailer.truck.control(gear=-1)

    truck_trailer.bng.control.resume()

    current_idx = 0
    route_points = truck_trailer.route

    while(True):
        # Llama a read_sensors en modo reversa = True para levantar la Reverse Cam
        sensores = truck_trailer.read_sensors(is_reverse=True)

        velocidad = sensores["v1"]
        psi_camion = sensores["psi_truck"]
        psi_trailer = sensores["psi_trailer"]
        delta_F2 = sensores["delta_F2"]
        pos_trailer = sensores["trailer_pos"]

        # Control Aditivo 
        steering, throttle, brake, current_idx, finished = control_aditivo_histeresis(
            pos_trailer, psi_trailer, psi_camion, delta_F2, velocidad, route_points, current_idx, lookahead=5.0
        )

        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0, gear=-1)

        # Si llegamos al final de la ruta generada (en reversa)
        if finished:
            print("Ruta de reversa completada. Frenando y devolviendo control manual (en Neutro)...")
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0, gear=0)
            break

    input('Presione enter cuando termine la simulación en reversa...')

    # truck_trailer.bng.close()

if __name__ == "__main__":
    main()
