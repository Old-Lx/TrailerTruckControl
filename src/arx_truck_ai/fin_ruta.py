"""
Utilidades de FIN DE RUTA compartidas por forward.py, backward.py y backwards_lqr.py.

Las usan igual los tres algoritmos (comparación justa en la tesis):

1. distancia_restante(): distancia que falta A LO LARGO de la ruta hasta el último punto,
   medida desde la proyección del vehículo (negativa si ya lo sobrepasó). No depende de qué
   tan lejos esté el vehículo del punto más cercano, así que funciona aunque llegue desviado
   lateralmente. (Antes el fin se decidía con "el punto más cercano es el último y estoy a
   menos de 2 m": si el vehículo llegaba desviado más de ~2 m, nunca se cumplía y seguía de largo.)

2. velocidad_limite_fin() / debe_frenar(): perfil de llegada (la velocidad objetivo baja como
   sqrt(2*a*d) al acercarse al final) y regla de frenado: frenar a fondo cuando la distancia de
   parada alcanza a la distancia que queda.

3. punto_objetivo_extendido(): objetivo virtual cuando queda menos ruta que el lookahead.
   La ruta se prolonga en línea recta a partir del ÚLTIMO PUNTO (no desde el vehículo), así el
   vehículo sigue siendo atraído lateralmente hacia el punto final. (Proyectarlo desde el
   vehículo solo alineaba el rumbo y lo dejaba seguir paralelo, desviado del punto final.)

4. detener_en_fin(): rutina de parada para main(). Frena hasta casi detenerse y entonces aplica
   el FRENO DE MANO, suelta el freno de servicio y pone neutro. Antes main() salía del bucle con
   brake=1 todavía aplicado: con la caja automática "arcade" de BeamNG, mantener el freno con el
   vehículo detenido engrana la reversa y el camión arranca hacia atrás.
"""
import math
import time

_CACHE = {}


def _acumuladas(ruta):
    """Longitud de arco acumulada de la ruta (cacheada: la ruta no cambia durante la corrida)."""
    clave = (id(ruta), len(ruta))
    acum = _CACHE.get(clave)
    if acum is None:
        acum = [0.0]
        for i in range(1, len(ruta)):
            acum.append(acum[-1] + math.hypot(ruta[i][0] - ruta[i - 1][0], ruta[i][1] - ruta[i - 1][1]))
        _CACHE.clear()
        _CACHE[clave] = acum
    return acum


def indice_segmento(ruta, idx):
    """Índice del segmento (idx, idx+1) válido: en el último punto se usa el último segmento."""
    return min(max(int(idx), 0), len(ruta) - 2)


def distancia_restante(ruta, idx, pos):
    """
    Distancia [m] a lo largo de la ruta desde la proyección de `pos` sobre el segmento
    (idx, idx+1) hasta el último punto. Es NEGATIVA si el vehículo ya pasó el último punto.
    """
    n = len(ruta)
    if n < 2:
        return 0.0
    acum = _acumuladas(ruta)
    i = indice_segmento(ruta, idx)
    ax, ay = ruta[i][0], ruta[i][1]
    bx, by = ruta[i + 1][0], ruta[i + 1][1]
    seg = acum[i + 1] - acum[i]
    t = ((pos[0] - ax) * (bx - ax) + (pos[1] - ay) * (by - ay)) / seg if seg > 1e-9 else 0.0
    if i < n - 2:
        t = min(max(t, 0.0), seg)   # en tramos intermedios la proyección se acota al segmento
    return (acum[-1] - acum[i]) - t


def velocidad_limite_fin(d_rem, a_perfil, v_final):
    """
    Perfil de aproximación: v = sqrt(2*a*d), sin bajar de v_final mientras quede ruta
    (para no quedarse corto) y 0 cuando ya no queda.
    """
    if d_rem <= 0.0:
        return 0.0
    return max(v_final, math.sqrt(2.0 * a_perfil * d_rem))


def debe_frenar(d_rem, velocidad, a_frenado, margen):
    """True cuando la distancia de parada a freno máximo alcanza la distancia que queda."""
    v = abs(velocidad)
    return d_rem <= (v * v) / (2.0 * a_frenado) + margen


def punto_objetivo_extendido(ruta, pos, lookahead):
    """
    Objetivo virtual a distancia `lookahead` del vehículo sobre la prolongación recta de la
    ruta, anclada en el último punto con la dirección del último segmento no degenerado.
    """
    n = len(ruta)
    pl = ruta[-1]
    ux = uy = 0.0
    for j in range(n - 1, 0, -1):                     # último segmento de longitud no nula
        dx = ruta[j][0] - ruta[j - 1][0]
        dy = ruta[j][1] - ruta[j - 1][1]
        nrm = math.hypot(dx, dy)
        if nrm > 1e-9:
            ux, uy = dx / nrm, dy / nrm
            break
    rx, ry = pl[0] - pos[0], pl[1] - pos[1]           # vehículo -> último punto
    b = rx * ux + ry * uy
    c = rx * rx + ry * ry - lookahead * lookahead
    disc = b * b - c
    s = (-b + math.sqrt(disc)) if disc > 0.0 else -b  # abscisa sobre la prolongación (desde el último punto)
    s = max(s, 0.0)
    return (pl[0] + ux * s, pl[1] + uy * s)


def _indice_cercano(ruta, pos, desde):
    mejor, idx = float('inf'), desde
    for i in range(max(0, desde), min(desde + 100, len(ruta))):
        d = math.hypot(pos[0] - ruta[i][0], pos[1] - ruta[i][1])
        if d < mejor:
            mejor, idx = d, i
    return idx


def _leer(truck_trailer, is_reverse):
    try:
        return truck_trailer.read_sensors(is_reverse=is_reverse, completo=False)
    except TypeError:                                  # TruckTrailer sin lectura liviana
        return truck_trailer.read_sensors(is_reverse=is_reverse)


def detener_en_fin(truck_trailer, ruta, idx, clave_pos, steering=0.0, is_reverse=False, gear=None,
                   v_detenido=0.3, timeout=20.0):
    """
    Rutina de parada para main(), una vez que el controlador declaró el fin de ruta.

    1. Freno a fondo (volante fijo en el último comando) hasta que |v| < v_detenido.
    2. Freno de mano, freno de servicio SUELTO, acelerador en 0 y neutro. Soltar el freno de
       servicio evita que la caja automática de BeamNG engrane la reversa al quedar detenido.

    En modo síncrono (truck_trailer.avanzar existe y avanza la simulación) la propia rutina avanza
    la simulación mientras frena; `timeout` se mide entonces en tiempo de simulación.

    Devuelve (sobrepaso, distancia_al_punto_final) medidos con el punto de control
    `clave_pos` ('truck_pos' en marcha adelante, 'trailer_pos' en reversa):
    sobrepaso > 0 significa que se pasó del último punto, < 0 que se detuvo antes.
    """
    extra = {} if gear is None else {'gear': gear}
    avanzar = getattr(truck_trailer, 'avanzar', None)
    sincrono = bool(getattr(truck_trailer, 'dt_control', None))     # None en modo libre
    paso = 1.0 / 60.0
    t0 = time.time()
    t_sim = 0.0
    sens = _leer(truck_trailer, is_reverse)
    while abs(sens['v1']) > v_detenido:
        if (t_sim if sincrono else time.time() - t0) >= timeout:
            break
        truck_trailer.truck.control(steering=steering, throttle=0.0, brake=1.0, parkingbrake=0, **extra)
        if sincrono:
            avanzar()
            t_sim += paso
        sens = _leer(truck_trailer, is_reverse)
    truck_trailer.truck.control(steering=steering, throttle=0.0, brake=0.0, parkingbrake=1, gear=0)
    if sincrono:
        for _ in range(30):                               # 0.5 s de simulación para asentar el freno de mano
            avanzar()
    else:
        time.sleep(0.5)
    sens = _leer(truck_trailer, is_reverse)
    pos = sens[clave_pos]
    i = _indice_cercano(ruta, pos, max(0, idx - 5))
    sobrepaso = -distancia_restante(ruta, i, pos)
    d_fin = math.hypot(pos[0] - ruta[-1][0], pos[1] - ruta[-1][1])
    return sobrepaso, d_fin
