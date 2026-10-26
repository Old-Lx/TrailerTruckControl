"""
main.py - Lanzador de experimentos del Capítulo 5.

1. Elige la ruta (TIPO_RUTA) y el algoritmo abajo y ejecuta, desde la raíz del repositorio:
       python src/main.py
   Cada corrida escribe telemetria_<algoritmo>_<ruta>.csv/.json y ruta_<ruta>.csv en la carpeta actual.
2. Con todas las corridas hechas:  python src/evaluacion_resultados.py --datos . --salida figuras/cap5
"""
from arx_truck_ai import truck_trailer
from arx_truck_ai import forward as fwd, backward as bwd, backwards_lqr as bwd_lqr

# 0 recta_x | 1 recta_y | 2 recta_diagonal | 3 parabola_x | 4 parabola_y | 5 seno_x | 6 seno_y
truck_trailer.TIPO_RUTA = 0

# Modo de ejecución: síncrono (registro a 60 Hz en tiempo de simulación, control a 60/8 = 7.5 Hz)
truck_trailer.MODO_SINCRONO = True
truck_trailer.DECIMACION_CONTROL = 8

ALGORITMO = 'lqr'   # 'forward' | 'backward' | 'lqr'

if __name__ == "__main__":
    {'forward': fwd.main, 'backward': bwd.main, 'lqr': bwd_lqr.main}[ALGORITMO]()
