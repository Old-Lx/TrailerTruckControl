import math

from arx_truck_ai import bng_open
from beamngpy import Vehicle
import numpy as np
import time

def control_proporcional(pos_actual, yaw_actual, velocidad, ruta, current_idx, lookahead=5.0):
    """
    Algoritmo de control de seguimiento de trayectoria básico usando Pure Pursuit / Proporcional.
    Retorna (steering, throttle, brake, next_idx, finished).

    Cada idx es un índice en el arreglo de puntos de la ruta.
    """
    if len(ruta) == 0:
        return 0.0, 0.0, 1.0, current_idx, True

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

    # Calcular el error de orientación (heading error)
    dx = target_pt[0] - pos_actual[0]
    dy = target_pt[1] - pos_actual[1]
    yaw_objetivo = math.atan2(dy, dx)

    raw_error = yaw_objetivo - yaw_actual
    # Normaliza el error a [-pi, pi]
    yaw_error = (raw_error + math.pi) % (2 * math.pi) - math.pi

    # Ganancia proporcional para la dirección
    kp_steer = 1.2
    # El simulador toma valores positivos de volante (steering > 0) para girar a la derecha. 
    # Un "yaw_error" positivo significa que el objetivo está a la izquierda (anti-horario),
    # por lo tanto, debemos invertir el signo del volante para girar correctamente.
    steering = -kp_steer * yaw_error
    # La máxima dirección de BeamNG entra en un rango de [-1, 1]
    steering = np.clip(steering, -1.0, 1.0)

    # Control de velocidad proporcional
    velocidad_objetivo = 6.0 # m/s
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

    return steering, throttle, brake, closest_idx, finished

def main():
    truck_trailer, orig = bng_open.main()

    # Posición de origen para el vehículo

    truck_trailer.bng.control.pause()
    truck_trailer.bng.settings.set_deterministic(60)

    script = []

    points = []
    point_color = (0, 0, 0, 0.1)
    sphere_coordinates = []
    sphere_radii = []
    sphere_colors = []

    # Ruta senoidal
    for i in range(2400):
        node = {
            #  Calculate the position as a sine curve that makes the truck
            #  drive from left to right. The z-coordinate is not calculated in
            #  any way because `ai.set_script` by default makes the polyline to
            #  follow cling to the ground, meaning the z-coordinate will be
            #  filled in automatically.
            "x": 4 * np.sin(np.radians(i)) + orig[0],
            "y": i * 0.2 + orig[1],
            "z": orig[2],
            #  Calculate timestamps for each node such that the speed between
            #  points has a sinusoidal variance to it.
            "t": (2 * i + (np.abs(np.sin(np.radians(i)))) * 64) / 64,
        }
        script.append(node)
        points.append((node["x"], node["y"], node["z"]))

        if i % 10 == 0:
            sphere_coordinates.append((node["x"], node["y"], node["z"]))
            sphere_radii.append(np.abs(np.sin(np.radians(i))) * 0.25)
            sphere_colors.append((np.sin(np.radians(i)), 0, 0, 0.8))

    ''' Esto añade las esferas al simulador, probablemente esto se borre porque la ruta debe verse sólo en la cámara 
    truck_trailer.bng.debug.add_spheres(
        sphere_coordinates, sphere_radii, sphere_colors, cling=True, offset=0.1
    )
    truck_trailer.bng.debug.add_polyline(points, point_color, cling=True, offset=0.1)
    '''

    # truck_trailer.bng.traffic.spawn()

    # Make the truck's AI span the map
    # truck_trailer.truck.ai.set_script(script)

    # for i in range(65):
    #    truck_trailer.bng.control.step(60)
    truck_trailer.bng.control.resume()

    current_idx = 0
    route_points = truck_trailer.route

    while(True):
        sensores = truck_trailer.read_sensors()

        pos_camion = sensores["truck_pos"]
        yaw_camion = sensores["psi_truck"]
        v1 = sensores["v1"]

        # Calcula el control proporcional simple para continuar por la ruta generada
        steering, throttle, brake, current_idx, finished = control_proporcional(
            pos_camion, yaw_camion, v1, route_points, current_idx, lookahead=6.0
        )

        # Aplicar el comando al vehículo en el simulador
        truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0)

        # Si llegamos al final de la trayectoria, detenemos el control autónomo
        if finished:
            print("Ruta completada. Frenando y devolviendo control manual...")
            # Se resetean los controles para dejar al usuario al mando
            truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=0)
            break

    input('Presione enter cuando termine la simulación...')

    # Disconnect BeamNG
    #truck_trailer.bng.disconnect()

    # Or close the simulator
    truck_trailer.bng.close()

if __name__ == "__main__":
    main()