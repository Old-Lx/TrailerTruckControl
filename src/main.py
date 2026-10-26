"""
main.py - Lanzador de experimentos del Capítulo 5.

1. Elige la ruta (TIPO_RUTA), el algoritmo y las opciones abajo y ejecuta, desde la raíz del repositorio:
       python src/main.py
   Cada corrida escribe telemetria_<algoritmo>_<ruta>.csv/.json y ruta_<ruta>.csv en la carpeta actual.
   El .json guarda las opciones usadas (ley de dirección, prealimentación, ...).
2. Con todas las corridas hechas:  python src/evaluacion_resultados.py --datos . --salida figuras/cap5

Para COMPARAR variantes (p. ej. LQR con y sin prealimentación) córrelas en carpetas distintas, porque el
nombre del archivo no incluye la variante:
       mkdir corridas\\lqr_sin_ff ; cd corridas\\lqr_sin_ff ; python ..\\..\\src\\main.py
y evalúa cada carpeta con --datos.
"""
from arx_truck_ai import truck_trailer
from arx_truck_ai import forward as fwd, backward as bwd, backwards_lqr as bwd_lqr

# 0 recta_x | 1 recta_y | 2 recta_diagonal | 3 parabola_x | 4 parabola_y | 5 seno_x | 6 seno_y
# (recta_y, parabola_y y recta_diagonal empiezan desalineadas a propósito: prueba de captura)
truck_trailer.TIPO_RUTA = 0

# Modo de ejecución: síncrono (registro a 60 Hz en tiempo de simulación, control a 60/8 = 7.5 Hz)
truck_trailer.MODO_SINCRONO = True
truck_trailer.DECIMACION_CONTROL = 8

ALGORITMO = 'lqr'   # 'forward' | 'backward' | 'lqr'

# ---- Opciones de los controladores (valores por defecto = los del módulo) ----------------------------
# Marcha adelante: 'proporcional' (ley validada hasta oct-2026) | 'pure_pursuit' (Pure Pursuit clásico)
fwd.LEY_DIRECCION = 'proporcional'
# LQR (ver MEJORAS DE OCT-2026 en backwards_lqr.py)
bwd_lqr.PREALIMENTACION_CURVATURA = True    # False = LQR hasta oct-2026 (regula hacia x = 0)
bwd_lqr.PROGRAMACION_EN_V = True            # False = K fija en V0 = 2 m/s
bwd_lqr.MODO_INTEGRADOR = 'original'        # 'original' | 'corregido'

if __name__ == "__main__":
    {'forward': fwd.main, 'backward': bwd.main, 'lqr': bwd_lqr.main}[ALGORITMO]()
