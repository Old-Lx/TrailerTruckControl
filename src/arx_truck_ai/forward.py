import math
from arx_truck_ai.debugging_sys import (registrar_muestra, generar_grafico_evaluacion, inicializar_telemetria,
                                       finalizar_telemetria)
from arx_truck_ai import truck_trailer as tt
from arx_truck_ai import bng_open
from arx_truck_ai import fin_ruta as fr
from arx_truck_ai.fin_ruta import (distancia_restante, velocidad_limite_fin, debe_frenar,
                                   punto_objetivo_extendido, indice_segmento, detener_en_fin, ControlVelocidad)
import numpy as np

# ---------------------------------------------------------------------------
# Fin de ruta (ver fin_ruta.py). El punto que debe detenerse en el final es truck_pos.
V_CRUCERO = 2.0          # m/s   velocidad de crucero (la misma que en reversa, para comparar en el Cap. 5;
                         #       hasta oct-2026 era 6.0)
A_APROXIMACION = 0.6     # m/s²  desaceleración del perfil de llegada: v_obj = sqrt(2*a*d_restante)
V_LLEGADA = 1.0          # m/s   velocidad mínima del perfil mientras quede ruta
A_FRENADO = 2.0          # m/s²  desaceleración que se ASUME con freno a fondo (conservadora:
                         #       si el camión frena más fuerte, se detiene un poco antes del final)
MARGEN_PARADA = 0.5      # m     margen extra antes del punto final
# ---------------------------------------------------------------------------
# Ley de dirección:
#   'proporcional'  steering = -KP_STEER * eta                          (ley validada en BeamNG hasta oct-2026)
#   'pure_pursuit'  delta = atan(2*L1*sin(eta)/L);  steering = -delta / PHI_MAX_RAD   (Pure Pursuit clásico)
# eta es el ángulo entre el rumbo del tractor y la recta al punto objetivo; L la distancia a ese punto.
# Para ángulos pequeños las dos son proporcionales a eta; con L = 12 m (lookahead a 2 m/s) la ganancia
# equivalente de Pure Pursuit es 2*L1/(L*PHI_MAX) = 0.81, menor que KP_STEER = 1.2.
LEY_DIRECCION = 'proporcional'
KP_STEER = 1.2
L1_TRACTOR = 2.93        # m    batalla efectiva del tractor (identificada, ver backwards_lqr.py)
PHI_MAX_RAD = 0.6        # rad  ángulo de rueda con |steering| = 1 (supuesto, mismo que el LQR)
KP_VELOCIDAD = 0.5       # ganancia proporcional del control de velocidad (fin_ruta.ControlVelocidad)

def control_proporcional(pos_actual, yaw_actual, truck_roll, truck_roll_rate, velocidad, ruta, current_idx, min_lookahead=5.0, max_lookahead=15.0, k_lookahead_gain=3.5,
                         ctrl_vel=None, dt=None):
    """
    Algoritmo de control de seguimiento de trayectoria básico usando Pure Pursuit / Proporcional.
    Implementa Lookahead Dinámico basado en la velocidad (con objetivo virtual extrapolado en el tramo
    final de la ruta) y Active Yaw/Roll Control limitando adherencia.
    Retorna (steering, throttle, brake, next_idx, finished, error_lateral, yaw_error).

    Cada idx es un índice en el arreglo de puntos de la ruta.
    """
    if not len(ruta):
        return 0.0, 0.0, 1.0, current_idx, True, 0.0, 0.0

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

    # Busca el punto "lookahead" a una distancia de la posición actual del CAMIÓN
    # Por defecto apuntamos al último punto de la ruta en caso de que estemos cerca del final
    target_idx = len(ruta) - 1
    found_target = False

    for i in range(closest_idx, len(ruta)):
        pt = ruta[i]
        dist = math.hypot(pos_actual[0] - pt[0], pos_actual[1] - pt[1])
        if dist >= lookahead:
            target_idx = i
            found_target = True
            break

    target_pt = ruta[target_idx]

    # Si estamos en el tramo final y la ruta se acaba (queda menos ruta que el lookahead),
    # el objetivo se toma sobre la prolongación recta del último segmento ANCLADA EN EL ÚLTIMO
    # PUNTO, a distancia `lookahead` del camión. Así el lookahead nunca se reduce a 0 y el camión
    # sigue siendo atraído lateralmente hacia el punto final. (Proyectar el objetivo desde el
    # propio camión solo alineaba el rumbo: el camión seguía paralelo a la ruta, desviado, y se
    # alejaba del punto final.) Es el mismo objetivo virtual que usan los dos algoritmos de reversa.
    if not found_target and len(ruta) > 1:
        target_pt = punto_objetivo_extendido(ruta, pos_actual, lookahead)

    # Distancia que falta hasta el final, medida A LO LARGO de la ruta (negativa si ya se pasó).
    d_restante = distancia_restante(ruta, closest_idx, pos_actual)

    # Cálculo del Error Lateral (cross-track error). En el último punto se usa el último
    # segmento (con el punto final repetido el segmento tenía longitud 0 y rumbo indefinido).
    idx_seg = indice_segmento(ruta, closest_idx)
    pt_actual_tr = ruta[idx_seg]
    pt_siguiente_tr = ruta[idx_seg + 1]
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

    # Atenuación del volante por balanceo (roll) de la suspensión
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

    # El simulador toma valores positivos de volante (steering > 0) para girar a la derecha.
    # Un "yaw_error" positivo significa que el objetivo está a la izquierda (anti-horario),
    # por lo tanto, debemos invertir el signo del volante para girar correctamente.
    if LEY_DIRECCION == 'pure_pursuit':
        L_objetivo = max(math.hypot(dx, dy), 1e-3)          # distancia real al punto objetivo
        delta_rueda = math.atan(2.0 * L1_TRACTOR * math.sin(yaw_error) / L_objetivo)
        steering = -(delta_rueda / PHI_MAX_RAD) * atenuacion_dinamica
    else:
        steering = -KP_STEER * yaw_error * atenuacion_dinamica
    # La máxima dirección de BeamNG entra en un rango de [-1, 1]
    steering = np.clip(steering, -1.0, 1.0)

    # Control de velocidad proporcional, con perfil de llegada: cerca del final la velocidad
    # objetivo baja como sqrt(2*a*d_restante) para llegar despacio y no pasarse del punto final.
    velocidad_objetivo = min(V_CRUCERO, velocidad_limite_fin(d_restante, A_APROXIMACION, V_LLEGADA))
    
    # Si la atenuación está muy activa (mucho roll), forzamos una reducción de la velocidad objetivo
    # como un sistema de control de estabilidad activo (ESC) para devolver tracción al asfalto.
    velocidad_objetivo *= atenuacion_dinamica
    
    # Control de velocidad común (fin_ruta.ControlVelocidad: PI o la ley 'original')
    if ctrl_vel is None:
        ctrl_vel = ControlVelocidad(KP_VELOCIDAD, V_CRUCERO, modo='original')
    throttle, brake = ctrl_vel(velocidad_objetivo, velocidad, dt)

    # Verificar si hemos llegado al final de la ruta: frenar a fondo cuando la distancia de parada
    # alcanza a la que queda (o si ya se pasó). Se mide a lo largo de la ruta, así que funciona
    # aunque el camión llegue desviado lateralmente (antes exigía estar a < 2 m del último punto).
    finished = False
    if debe_frenar(d_restante, velocidad, A_FRENADO, MARGEN_PARADA):
        finished = True
        return steering, 0.0, 1.0, closest_idx, finished, error_lateral, yaw_error

    return steering, throttle, brake, closest_idx, finished, error_lateral, yaw_error

def main():
    direction = 'fwd'
    truck_trailer, orig = bng_open.main(direction)
    truck_trailer.iniciar_simulacion()

    current_idx = 0
    route_points = truck_trailer.route
    nombre_ruta = truck_trailer.nombre_ruta

    # Telemetría (entrada de evaluacion_resultados.py): una fila por paso de simulación
    inicializar_telemetria(f"telemetria_forward_{nombre_ruta}.csv", puntos_ruta=route_points, sentido=+1,
                           punto_control='tractor', algoritmo='forward', ruta=nombre_ruta,
                           tipo_ruta=tt.TIPO_RUTA, modo='sincrono' if tt.MODO_SINCRONO else 'libre',
                           frecuencia_simulacion=tt.FRECUENCIA_SIMULACION, pasos_por_segundo=tt.PASOS_POR_SEGUNDO,
                           frecuencia_control=tt.FRECUENCIA_SIMULACION / truck_trailer.decimacion_control
                           if tt.MODO_SINCRONO else None,
                           ley_direccion=LEY_DIRECCION, v_crucero=V_CRUCERO,
                           control_velocidad=fr.MODO_CONTROL_VELOCIDAD)

    # Estado del lazo
    previous_truck_roll = None   # None: en el primer ciclo la tasa de roll es 0 (no se deriva contra 0)
    t0 = None                 # t = 0 en la primera muestra
    t_ultimo_control = None
    steering, throttle, brake = 0.0, 0.0, 0.0
    ctrl_vel = ControlVelocidad(KP_VELOCIDAD, V_CRUCERO)
    error_lateral = yaw_error = None
    finished = False
    resultado, info_fin = 'completada', {}
    ciclo = 0

    while True:
        # En modo síncrono el control se recalcula cada `decimacion_control` pasos y la telemetría se
        # registra en todos; en modo libre ambos ocurren en cada iteración.
        es_ciclo_control = (ciclo % truck_trailer.decimacion_control == 0)
        sensores = truck_trailer.read_sensors(completo=es_ciclo_control)
        t = sensores['t_sim']
        if t0 is None:
            t0 = t
        t_rel = t - t0

        pos_camion = sensores["truck_pos"]
        yaw_camion = sensores["psi_truck"]
        v1 = sensores["v1"]
        truck_roll = sensores["truck_roll"]

        if es_ciclo_control:
            dt = (t - t_ultimo_control) if t_ultimo_control is not None else (truck_trailer.dt_control or 0.1)
            dt = max(dt, 1e-4)
            t_ultimo_control = t

            # Calcular la rotación/inercia de balanceo (tasa de roll)
            truck_roll_rate = 0.0 if previous_truck_roll is None else (truck_roll - previous_truck_roll) / dt
            previous_truck_roll = truck_roll

            # Control proporcional (Pure Pursuit) con lookahead dinámico y control amortiguado (Active Yaw)
            steering, throttle, brake, current_idx, finished, error_lateral, yaw_error = control_proporcional(
                pos_camion, yaw_camion, truck_roll, truck_roll_rate, v1, route_points, current_idx,
                ctrl_vel=ctrl_vel, dt=dt
            )
            # Aplicar el comando al vehículo en el simulador (se mantiene hasta la próxima actualización)
            truck_trailer.truck.control(steering=steering, throttle=throttle, brake=brake, parkingbrake=0)

        # Telemetría: el tick de fin de ruta es un comando de frenado, no una muestra de seguimiento
        if not finished:
            registrar_muestra(t_rel, sensores, steering, control_actualizado=es_ciclo_control,
                              error_lateral_ctrl=error_lateral, error_angular_ctrl=yaw_error,
                              throttle=throttle, brake=brake)

        if finished:
            print("Fin de ruta: frenando hasta detener el camión...")
            # Frena hasta detenerse y deja el camión con freno de mano, sin freno de servicio y en
            # neutro (con el freno de servicio aplicado y el camión detenido, la caja automática
            # de BeamNG engrana la reversa y el camión arranca hacia atrás).
            sobrepaso, d_fin = detener_en_fin(truck_trailer, route_points, current_idx, 'truck_pos',
                                              steering=steering, is_reverse=False)
            print(f"Camión detenido. Sobrepaso a lo largo de la ruta: {sobrepaso:+.2f} m "
                  f"(+ = se pasó del punto final) | distancia al punto final: {d_fin:.2f} m")
            resultado, info_fin = 'completada', dict(sobrepaso_m=sobrepaso, distancia_final_m=d_fin)
            break
        if t_rel > tt.DURACION_MAXIMA_S:
            print(f"Tiempo máximo del experimento ({tt.DURACION_MAXIMA_S:.0f} s) alcanzado: deteniendo...")
            sobrepaso, d_fin = detener_en_fin(truck_trailer, route_points, current_idx, 'truck_pos',
                                              steering=steering, is_reverse=False)
            resultado, info_fin = 'tiempo_maximo', dict(sobrepaso_m=sobrepaso, distancia_final_m=d_fin)
            break

        truck_trailer.avanzar()      # modo síncrono: un paso de simulación (1/60 s); modo libre: nada
        ciclo += 1

    finalizar_telemetria(resultado, **info_fin)
    truck_trailer.liberar_simulacion()
    generar_grafico_evaluacion(titulo='Evaluación Controlador Proporcional (Marcha Adelante)')
    input('Presione enter cuando termine la simulación...')

    # Disconnect BeamNG
    #truck_trailer.bng.disconnect()

    # Or close the simulator
    truck_trailer.bng.close()

if __name__ == "__main__":
    main()
