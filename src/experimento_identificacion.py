"""
experimento_identificacion.py - Experimento en BeamNG para identificar la planta (Capítulo 3).

Conduce el camión con tráiler EN MARCHA ADELANTE a velocidad constante con una secuencia de volante en lazo
abierto (escalones y senoides) y registra, en cada paso de simulación (60 Hz):
    comando de volante, volante real (sensor Electrics), velocidad, rumbos del tractor y del remolque,
    articulación y posiciones de los puntos de control.
Marcha adelante porque es estable en lazo abierto; los parámetros que se identifican (L1/PHI, L2, desplazamiento
del enganche, DP y la dinámica del volante) son geométricos o del actuador, y valen igual en reversa.

Uso (desde la raíz del repositorio, con BeamNG.tech configurado en .env):
    python src/experimento_identificacion.py
Escribe en la carpeta actual identificacion_planta.csv (+ .json) y luego se analiza con:
    python src/identificar_planta.py identificacion_planta.csv --salida figuras/cap3
"""
import csv
import json
import math
import time

from arx_truck_ai import bng_open
from arx_truck_ai import truck_trailer as tt
from arx_truck_ai import fin_ruta as fr

V_OBJETIVO = 2.0             # m/s
KP_VELOCIDAD = 0.5           # control de velocidad común (fin_ruta.ControlVelocidad), como en forward.py
ARTICULACION_MAXIMA = 1.0    # rad: corte de seguridad
ARCHIVO = 'identificacion_planta.csv'

# Secuencia de volante: (duración [s], tipo, amplitud, frecuencia [Hz])
SECUENCIA = [
    (12.0, 'cte', 0.0, None),      # arranque y asentamiento en recta
    (8.0, 'cte', 0.2, None), (8.0, 'cte', 0.0, None),
    (8.0, 'cte', -0.2, None), (8.0, 'cte', 0.0, None),
    (6.0, 'cte', 0.4, None), (8.0, 'cte', 0.0, None),
    (6.0, 'cte', -0.4, None), (8.0, 'cte', 0.0, None),
    (40.0, 'seno', 0.25, 0.05),    # 2 ciclos
    (20.0, 'seno', 0.25, 0.10),
    (10.0, 'seno', 0.20, 0.20),
    (5.0, 'seno', 0.15, 0.40),
    (6.0, 'cte', 0.0, None),
]

CABECERA = ['t', 'steering_cmd', 'volante_input', 'volante_real_deg', 'velocidad',
            'psi_tractor', 'psi_trailer', 'delta_F2',
            'x_tractor', 'y_tractor', 'x_trailer', 'y_trailer', 'throttle', 'brake']


def volante_programado(t):
    """Comando de volante de la secuencia en el instante t (None al terminar)."""
    t0 = 0.0
    for dur, tipo, amp, f in SECUENCIA:
        if t < t0 + dur:
            return amp if tipo == 'cte' else amp * math.sin(2 * math.pi * f * (t - t0))
        t0 += dur
    return None


def main():
    truck_trailer, _ = bng_open.main('fwd')
    truck_trailer.iniciar_simulacion()
    filas = []
    resultado = 'completado'
    t0 = None
    ctrl_vel = fr.ControlVelocidad(KP_VELOCIDAD, V_OBJETIVO)
    while True:
        s = truck_trailer.read_sensors(completo=False)
        t0 = s['t_sim'] if t0 is None else t0
        t = s['t_sim'] - t0
        cmd = volante_programado(t)
        if cmd is None:
            break
        if abs(s['delta_F2']) > ARTICULACION_MAXIMA:
            resultado = 'corte_por_articulacion'
            break
        throttle, brake = ctrl_vel(V_OBJETIVO, s['v1'], 1.0 / tt.FRECUENCIA_SIMULACION)
        truck_trailer.truck.control(steering=cmd, throttle=throttle, brake=brake, parkingbrake=0)
        filas.append([t, cmd, s.get('volante_input'), s.get('volante_real_deg'), s['v1'],
                      s['psi_truck'], s['psi_trailer'], s['delta_F2'],
                      s['truck_pos'][0], s['truck_pos'][1], s['trailer_pos'][0], s['trailer_pos'][1],
                      throttle, brake])
        truck_trailer.avanzar()      # modo síncrono: un paso de 1/60 s

    truck_trailer.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=1, gear=0)
    with open(ARCHIVO, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(CABECERA)
        for fila in filas:
            w.writerow(['' if v is None else v for v in fila])
    meta = dict(resultado=resultado, muestras=len(filas), v_objetivo=V_OBJETIVO, sentido=+1,
                modo='sincrono' if tt.MODO_SINCRONO else 'libre', frecuencia_simulacion=tt.FRECUENCIA_SIMULACION,
                pasos_por_segundo=tt.PASOS_POR_SEGUNDO,
                secuencia=SECUENCIA, control_velocidad=fr.MODO_CONTROL_VELOCIDAD, fecha=time.strftime('%Y-%m-%d %H:%M:%S'))
    with open(ARCHIVO.replace('.csv', '.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[identificación] {ARCHIVO}: {len(filas)} muestras, resultado: {resultado}")
    truck_trailer.liberar_simulacion()
    input('Presione enter para cerrar...')
    truck_trailer.bng.close()


if __name__ == "__main__":
    main()
