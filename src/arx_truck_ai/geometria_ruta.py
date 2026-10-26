"""
Geometría de la ruta: curvatura a lo largo de la polilínea (la usa la prealimentación del LQR).

La curvatura de la ruta en el punto i es el cambio de rumbo entre los segmentos (i-1, i) e (i, i+1)
dividido por la longitud media de ambos:  kappa_i = wrap(h_i - h_{i-1}) / ((l_{i-1} + l_i) / 2).
Positiva si la ruta gira a la izquierda (sentido antihorario) recorriéndola en su sentido de avance.
Se suaviza con una media móvil de VENTANA_SUAVIZADO_M metros para no amplificar la discretización.
Antes del inicio y después del final (prolongación recta) la curvatura es 0.
"""
import math

import numpy as np

from arx_truck_ai.fin_ruta import distancia_restante

VENTANA_SUAVIZADO_M = 3.0

_CACHE = {}


def _perfil(ruta):
    """(s, kappa): longitud de arco de cada punto y curvatura suavizada (cacheado por ruta)."""
    clave = (id(ruta), len(ruta))
    perfil = _CACHE.get(clave)
    if perfil is None:
        r = np.asarray(ruta, dtype=np.float64)[:, :2]
        d = np.diff(r, axis=0)
        largo = np.hypot(d[:, 0], d[:, 1])
        s = np.concatenate(([0.0], np.cumsum(largo)))
        rumbo = np.arctan2(d[:, 1], d[:, 0])
        kappa = np.zeros(len(r))
        for i in range(1, len(r) - 1):
            l_media = 0.5 * (largo[i - 1] + largo[i])
            if l_media > 1e-9:
                kappa[i] = math.atan2(math.sin(rumbo[i] - rumbo[i - 1]), math.cos(rumbo[i] - rumbo[i - 1])) / l_media
        paso = max(float(np.median(largo)) if len(largo) else 1.0, 1e-6)
        n = max(1, int(round(VENTANA_SUAVIZADO_M / paso)))
        if n > 1:
            kappa = np.convolve(kappa, np.ones(n) / n, mode='same')
        perfil = (s, kappa)
        _CACHE.clear()
        _CACHE[clave] = perfil
    return perfil


def curvatura_adelante(ruta, idx, pos, adelanto_m=0.0):
    """
    Curvatura de la ruta [1/m] en el punto que está `adelanto_m` metros (medidos sobre la ruta) por
    delante de la proyección de `pos` sobre el segmento `idx`. 0 fuera de la ruta.
    """
    if len(ruta) < 3:
        return 0.0
    s, kappa = _perfil(ruta)
    s_q = (s[-1] - distancia_restante(ruta, idx, pos)) + adelanto_m
    if s_q <= s[0] or s_q >= s[-1]:
        return 0.0
    return float(np.interp(s_q, s, kappa))
