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
from arx_truck_ai import fin_ruta
from arx_truck_ai import forward as fwd, backward as bwd, backwards_lqr as bwd_lqr

# 0 recta_x | 1 recta_y | 2 recta_diagonal | 3 parabola_x | 4 parabola_y | 5 seno_x | 6 seno_y
# (recta_y, parabola_y y recta_diagonal empiezan desalineadas a propósito: prueba de captura)
truck_trailer.TIPO_RUTA = 0

# Modo de ejecución: síncrono (registro a 60 Hz en tiempo de simulación, control a 60/8 = 7.5 Hz)
truck_trailer.MODO_SINCRONO = True
truck_trailer.DECIMACION_CONTROL = 8
# Cuadros de BeamNG por segundo: 60 = valor validado. 120 acelera ~1.5 veces pero cambia ligeramente la
# simulación (BeamNG procesa las entradas una vez por cuadro): solo tras el A/B (ver truck_trailer.py).
truck_trailer.PASOS_POR_SEGUNDO = 60
# Telemetría cada n pasos: 1 = 60 Hz (0.32x tiempo real) | 2 = 30 Hz (0.47x) | 4 = 15 Hz (0.62x) | 8 = 7.5 Hz (0.73x).
# La física no cambia (ver truck_trailer.DECIMACION_TELEMETRIA); solo hay menos filas en el CSV.
truck_trailer.DECIMACION_TELEMETRIA = 8
# Cámaras con la ruta proyectada (solo visualización). False acelera la simulación; los datos no cambian.
truck_trailer.CAMARAS_ACTIVAS = True

ALGORITMO = 'backward'   # 'forward' | 'backward' | 'lqr'

# ---- Opciones de los controladores (valores por defecto = los del módulo) ----------------------------
# Control de velocidad, común a los tres: 'pi' (desde oct-2026) | 'original' (P con 0.1 fijo: ciclo límite)
fin_ruta.MODO_CONTROL_VELOCIDAD = 'pi'
# Marcha adelante: 'pure_pursuit' (Pure Pursuit clásico, por defecto desde el A/B del 04-nov) | 'proporcional'
fwd.LEY_DIRECCION = 'pure_pursuit'
# LQR (ver MEJORAS DE OCT-2026 en backwards_lqr.py)
bwd_lqr.PREALIMENTACION_CURVATURA = True    # False = LQR hasta oct-2026 (regula hacia x = 0)
bwd_lqr.PROGRAMACION_EN_V = True            # False = K fija en V0 = 2 m/s
bwd_lqr.MODO_INTEGRADOR = 'corregido'       # 'corregido' (anti-windup, por defecto) | 'original' (casi sin acción integral)

if __name__ == "__main__":
    {'forward': fwd.main, 'backward': bwd.main, 'lqr': bwd_lqr.main}[ALGORITMO]()
