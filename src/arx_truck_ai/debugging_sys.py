import atexit
import csv
import json
import logging
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import time

# ==========================================
# MÓDULO DE DEBUGGING Y EVALUACIÓN
# ==========================================
logging.getLogger('beamngpy').setLevel(logging.WARNING)

# Diccionario global para almacenar el historial de datos en memoria para la gráfica
historial = {
    'tiempo': [],
    'x1_lat': [],
    'x2_ang': [],
    'x3_tij': [],
    'volante': []
}

# ------------------------------------------------------------------
# Exportación a CSV (entrada de evaluacion_resultados.py, Capítulo 5)
# ------------------------------------------------------------------
# Por cada corrida se escriben, en la carpeta de trabajo:
#   telemetria_<algoritmo>_<ruta>.csv   una fila por paso de simulación (60 Hz en modo síncrono)
#   telemetria_<algoritmo>_<ruta>.json  metadatos: algoritmo, ruta, modo, resultado, sobrepaso final...
#   ruta_<ruta>.csv                     la ruta de referencia completa (x, y)
#
# Las 9 primeras columnas son las de siempre (compatibilidad). Los errores de seguimiento se calculan
# AQUÍ con una única definición geométrica para los tres algoritmos (no dependen de cada controlador):
#   error_lateral  distancia con signo (+ = izquierda del sentido de la ruta) del PUNTO DE CONTROL a la
#                  ruta: el tractor en marcha adelante, el remolque en reversa.
#   error_angular  error angular del REMOLQUE: tangente de la ruta en el punto más cercano al remolque
#                  menos el rumbo de avance del remolque (su nariz en marcha adelante, su cola en reversa).
CSV_CABECERA = ['t', 'x_real', 'y_real', 'x_ref', 'y_ref',
                'error_lateral', 'error_angular', 'delta_F2', 'steering',
                'x_tractor', 'y_tractor', 'x_trailer', 'y_trailer', 'psi_tractor', 'psi_trailer', 'velocidad',
                'error_lateral_tractor', 'error_lateral_trailer',
                'error_lateral_ctrl', 'error_angular_ctrl', 'control_actualizado',
                'volante_input', 'volante_real_deg', 'throttle', 'brake']

archivo_csv_activo = None
_archivo_abierto = None      # archivo del CSV activo, abierto con búfer (ver _fila_csv)
_escritor_csv = None
_aviso_sin_csv_emitido = False
_sesion = {}          # estado de la corrida activa (ruta, sentido, metadatos, contadores)


def _envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


class SeguidorRuta:
    """Proyección de un punto sobre la ruta (polilínea) con búsqueda local incremental.

    proyectar(P) -> (error_lateral, rumbo_tangente, pie): distancia con signo (+ = izquierda del
    sentido de la ruta), rumbo de la tangente del segmento más cercano y punto pie de la proyección.
    El primer y el último segmento se prolongan (puntos antes del inicio o después del final).
    """

    def __init__(self, ruta, ventana_atras=10, ventana_adelante=80):
        r = np.asarray(ruta, dtype=np.float64)[:, :2]
        d = np.diff(r, axis=0)
        largo = np.hypot(d[:, 0], d[:, 1])
        validos = largo > 1e-9
        self.a = r[:-1][validos]
        self.d = d[validos] / largo[validos, None]
        self.largo = largo[validos]
        self.rumbo = np.arctan2(self.d[:, 1], self.d[:, 0])
        self.idx = None
        self.atras, self.adelante = ventana_atras, ventana_adelante

    def proyectar(self, P):
        P = np.asarray(P, dtype=np.float64)[:2]
        n = len(self.a)
        if self.idx is None:
            lo, hi = 0, n
        else:
            lo, hi = max(0, self.idx - self.atras), min(n, self.idx + self.adelante)
        a, d, L = self.a[lo:hi], self.d[lo:hi], self.largo[lo:hi]
        w = P - a
        t = w[:, 0] * d[:, 0] + w[:, 1] * d[:, 1]
        t_lim = np.clip(t, 0.0, L)
        if lo == 0:
            t_lim[0] = min(t[0], L[0])                 # antes del inicio: prolongación del primer segmento
        if hi == n:
            t_lim[-1] = max(t[-1], 0.0)                # después del final: prolongación del último
        pie = a + d * t_lim[:, None]
        dist = np.hypot(P[0] - pie[:, 0], P[1] - pie[:, 1])
        k = int(np.argmin(dist))
        self.idx = lo + k
        lat = d[k, 0] * (P[1] - a[k, 1]) - d[k, 1] * (P[0] - a[k, 0])
        return float(lat), float(self.rumbo[self.idx]), pie[k]


def _escribir_ruta(nombre_archivo_ruta, ruta):
    with open(nombre_archivo_ruta, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['x', 'y'])
        for p in ruta:
            w.writerow([float(p[0]), float(p[1])])


def inicializar_telemetria(nombre_archivo, puntos_ruta=None, sentido=+1, punto_control='tractor', **metadatos):
    """
    Crea (o sobrescribe) el CSV de la corrida con la cabecera estándar y lo deja activo.
    Llamar UNA vez por experimento, antes del bucle de control.

    puntos_ruta:    lista de puntos de la ruta (habilita los errores geométricos y escribe ruta_<nombre>.csv)
    sentido:        +1 marcha adelante, -1 reversa (define el rumbo de avance del remolque)
    punto_control:  'tractor' o 'trailer': qué punto se reporta en x_real/y_real/error_lateral
    metadatos:      se guardan en el .json de la corrida (algoritmo, ruta, modo, ...)
    """
    global archivo_csv_activo, _aviso_sin_csv_emitido, _sesion
    _cerrar_csv()
    archivo_csv_activo = nombre_archivo
    _aviso_sin_csv_emitido = False
    for k in historial:
        historial[k].clear()
    with open(archivo_csv_activo, 'w', newline='', encoding='utf-8') as f:
        csv.writer(f, lineterminator='\n').writerow(CSV_CABECERA)

    _sesion = dict(sentido=1 if sentido >= 0 else -1, punto_control=punto_control,
                   metadatos=dict(metadatos), n=0, t_ini=None, t_fin=None,
                   seg_tractor=None, seg_trailer=None)
    if puntos_ruta is not None and len(puntos_ruta) > 1:
        _sesion['seg_tractor'] = SeguidorRuta(puntos_ruta)
        _sesion['seg_trailer'] = SeguidorRuta(puntos_ruta)
        nombre_ruta = metadatos.get('ruta')
        if nombre_ruta:
            carpeta = os.path.dirname(os.path.abspath(nombre_archivo))
            archivo_ruta = os.path.join(carpeta, f"ruta_{nombre_ruta}.csv")
            _escribir_ruta(archivo_ruta, puntos_ruta)
            _sesion['metadatos']['archivo_ruta'] = os.path.basename(archivo_ruta)


def _fila_csv(fila):
    """Agrega una fila al CSV activo. El archivo queda abierto con búfer durante la corrida (antes se abría y
    cerraba en cada fila, 60 veces por segundo de simulación) y se cierra en finalizar_telemetria()."""
    global _archivo_abierto, _escritor_csv
    if _archivo_abierto is None:
        _archivo_abierto = open(archivo_csv_activo, 'a', newline='', encoding='utf-8')
        _escritor_csv = csv.writer(_archivo_abierto, lineterminator='\n')
    _escritor_csv.writerow(['' if v is None else v for v in fila])
    if _sesion.get('n', 0) % 600 == 0:          # cada 10 s de simulación: lo escrito queda en disco
        _archivo_abierto.flush()


@atexit.register
def _cerrar_csv():
    global _archivo_abierto, _escritor_csv
    if _archivo_abierto is not None:
        _archivo_abierto.close()
    _archivo_abierto = _escritor_csv = None


def registrar_muestra(t, sensores, steering, control_actualizado=True,
                      error_lateral_ctrl=None, error_angular_ctrl=None, throttle=None, brake=None):
    """
    Registra UNA muestra a partir del diccionario de read_sensors() y del comando de dirección aplicado.
    Calcula los errores geométricos uniformes (ver cabecera) y escribe la fila en el CSV activo.
    error_*_ctrl: las señales internas del controlador en ese instante (solo diagnóstico).
    throttle, brake: comandos de acelerador y freno aplicados (retenidos entre ciclos de control).
    """
    global _aviso_sin_csv_emitido
    tractor = sensores['truck_pos']
    trailer = sensores['trailer_pos']
    psi_tractor = float(sensores['psi_truck'])
    psi_trailer = float(sensores['psi_trailer'])
    delta = float(sensores['delta_F2'])
    v = float(sensores['v1'])

    lat_tr = lat_rm = e_ang = None
    pie_tr = pie_rm = None
    if _sesion.get('seg_tractor') is not None:
        lat_tr, _, pie_tr = _sesion['seg_tractor'].proyectar(tractor)
        lat_rm, rumbo_rm, pie_rm = _sesion['seg_trailer'].proyectar(trailer)
        rumbo_avance = psi_trailer + (math.pi if _sesion['sentido'] < 0 else 0.0)
        e_ang = _envolver(rumbo_rm - rumbo_avance)

    if _sesion.get('punto_control', 'tractor') == 'trailer':
        real, pie, lat = trailer, pie_rm, lat_rm
    else:
        real, pie, lat = tractor, pie_tr, lat_tr
    if lat is None:                       # sin ruta: se registran las señales del controlador
        lat, e_ang = error_lateral_ctrl, error_angular_ctrl
    x_ref, y_ref = (float(pie[0]), float(pie[1])) if pie is not None else (None, None)

    historial['tiempo'].append(t)
    historial['x1_lat'].append(lat if lat is not None else float('nan'))
    historial['x2_ang'].append(e_ang if e_ang is not None else float('nan'))
    historial['x3_tij'].append(delta)
    historial['volante'].append(float(steering))

    if archivo_csv_activo is None:
        if not _aviso_sin_csv_emitido:
            print("[telemetria] AVISO: registrar_muestra() se llamó sin inicializar_telemetria(); "
                  "no se está escribiendo ningún CSV.")
            _aviso_sin_csv_emitido = True
        return
    _fila_csv([t, float(real[0]), float(real[1]), x_ref, y_ref, lat, e_ang, delta, float(steering),
               float(tractor[0]), float(tractor[1]), float(trailer[0]), float(trailer[1]), psi_tractor, psi_trailer, v,
               lat_tr, lat_rm, error_lateral_ctrl, error_angular_ctrl, int(bool(control_actualizado)),
               sensores.get('volante_input'), sensores.get('volante_real_deg'),
               None if throttle is None else float(throttle), None if brake is None else float(brake)])
    _sesion['n'] = _sesion.get('n', 0) + 1
    if _sesion.get('t_ini') is None:
        _sesion['t_ini'] = t
    _sesion['t_fin'] = t


def finalizar_telemetria(resultado, **extra):
    """
    Cierra la corrida: escribe telemetria_<...>.json con los metadatos y el resultado
    ('completada', 'jackknife' o 'tiempo_maximo') e imprime un resumen.
    """
    if archivo_csv_activo is None:
        return
    _cerrar_csv()
    meta = dict(_sesion.get('metadatos', {}))
    meta.update(extra)
    n = _sesion.get('n', 0)
    t0, t1 = _sesion.get('t_ini'), _sesion.get('t_fin')
    meta.update(resultado=resultado, muestras=n, sentido=_sesion.get('sentido', 1),
                punto_control=_sesion.get('punto_control'), archivo_csv=os.path.basename(archivo_csv_activo),
                duracion_s=(t1 - t0) if (t0 is not None and t1 is not None) else None,
                frecuencia_media_hz=((n - 1) / (t1 - t0)) if (n > 1 and t1 > t0) else None,
                fecha=time.strftime('%Y-%m-%d %H:%M:%S'))
    archivo_json = os.path.splitext(archivo_csv_activo)[0] + '.json'
    with open(archivo_json, 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2, default=float)
    f_txt = f"{meta['frecuencia_media_hz']:.1f} Hz" if meta['frecuencia_media_hz'] else '-'
    print(f"[telemetria] {os.path.basename(archivo_csv_activo)}: {n} muestras, {f_txt}, resultado: {resultado}")


def generar_grafico_evaluacion(titulo='Evaluación de Respuesta del Controlador'):
    """
    Genera un panel de 3 gráficas (Subplots) usando matplotlib para evaluar 
    la respuesta transitoria del controlador al finalizar la simulación.
    """
    t = historial['tiempo']
    
    if len(t) == 0:
        print("No hay datos de telemetría para graficar.")
        return

    # Crear una figura con 3 subgráficos apilados
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(titulo, fontsize=14)

    # Gráfica 1: Errores de Seguimiento de Trayectoria (x1 y x2)
    ax1.plot(t, historial['x1_lat'], label='Error Lateral (m)', color='blue')
    ax1.plot(t, historial['x2_ang'], label='Error Angular (rad)', color='cyan', linestyle='--')
    ax1.set_ylabel('Errores Trayectoria')
    ax1.grid(True)
    ax1.legend(loc='upper right')

    # Gráfica 2: Ángulo de Articulación / Efecto Tijera (x3)
    ax2.plot(t, historial['x3_tij'], label='Articulación (rad)', color='red')
    ax2.axhline(y=0, color='black', linewidth=1) # Línea cero de referencia
    ax2.set_ylabel('Efecto Tijera')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    # Gráfica 3: Esfuerzo de Control (Volante)
    ax3.plot(t, historial['volante'], label='Comando Volante [-1, 1]', color='green')
    ax3.axhline(y=1.0, color='gray', linestyle=':') # Límite físico
    ax3.axhline(y=-1.0, color='gray', linestyle=':') # Límite físico
    ax3.set_ylabel('Esfuerzo de Control')
    ax3.set_xlabel('Tiempo de Simulación (s)')
    ax3.grid(True)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('respuesta_sistema.png') # Guarda la imagen genérica
    print("Gráfica guardada como 'respuesta_sistema.png'. Abriendo ventana...")
    plt.show() # Muestra la ventana interactiva

# Figuras explicativas del Capítulo 4 (no se llaman durante los experimentos)
def generar_diagrama_pure_pursuit_forward(output_path="./pure_pursuit_forward.png"):
    """
    Genera el esquema geométrico de Persecución Pura (Pure Pursuit) en marcha hacia adelante.
    Grafica la trayectoria, la pose del tractor, la distancia de mirada L, el ángulo eta
    y el arco de curvatura R.
    """
    # 1. Crear carpeta de destino si no existe
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # 2. Configurar la figura a alta resolución (300 DPI)
    fig, ax = plt.subplots(figsize=(8, 6), dpi=300)

    # 3. Parámetros geométricos de prueba
    x1, y1 = 0.0, 0.0         # Posición del eje trasero del tractor
    psi1 = np.radians(20)      # Orientación del tractor (20 grados)
    L1 = 3.8                   # Distancia entre ejes del tractor
    L = 6.0                    # Distancia de mirada (Lookahead)
    eta_deg = 35               # Ángulo relativo respecto al punto objetivo
    eta = np.radians(eta_deg)

    # Punto objetivo P_look
    x_g = x1 + L * np.cos(psi1 + eta)
    y_g = y1 + L * np.sin(psi1 + eta)

    # Radio de curvatura R
    R = L / (2.0 * np.sin(eta))

    # Centro de la circunferencia de curvatura (perpendicular a psi1)
    x_c = x1 - R * np.sin(psi1)
    y_c = y1 + R * np.cos(psi1)

    # 4. Trazar Trayectoria de Referencia (Curva suave)
    x_path = np.linspace(-2, 10, 200)
    y_path = 0.5 * x_path + 1.2 * np.sin(0.4 * x_path)
    ax.plot(x_path, y_path, 'k--', linewidth=2, label='Ruta de Referencia')

    # 5. Circunferencia del Lookahead (Radio L)
    theta_circle = np.linspace(0, 2 * np.pi, 200)
    ax.plot(x1 + L * np.cos(theta_circle), y1 + L * np.sin(theta_circle),
            color='gray', linestyle=':', linewidth=1.2, label=f'Circunferencia Lookahead ($L = {L}$m)')

    # 6. Arco de Curvatura R
    angle_start = np.arctan2(y1 - y_c, x1 - x_c)
    angle_end = np.arctan2(y_g - y_c, x_g - x_c)
    angles_arc = np.linspace(angle_start, angle_end, 100)
    ax.plot(x_c + R * np.cos(angles_arc), y_c + R * np.sin(angles_arc),
            color='crimson', linewidth=2.5, label=f'Arco de Incorporación ($R = {R:.2f}$m)')

    # 7. Cuerda L (Línea desde el tractor al punto objetivo)
    ax.plot([x1, x_g], [y1, y_g], color='blue', linestyle='-', linewidth=2, label='Cuerda $L$ (Lookahead)')

    # 8. Vector de orientación del tractor
    v_length = 3.0
    ax.arrow(x1, y1, v_length * np.cos(psi1), v_length * np.sin(psi1),
             head_width=0.3, head_length=0.4, fc='navy', ec='navy', linewidth=1.5)
    ax.plot([x1, x1 + v_length * np.cos(psi1)], [y1, y1 + v_length * np.sin(psi1)],
            color='navy', linewidth=2, label=r'Eje Longitudinal ($\psi_1$)')

    # 9. Puntos clave (Tractor y P_look)
    ax.scatter([x1], [y1], color='navy', s=120, zorder=5, label='Eje Trasero Tractor $(x_1, y_1)$')
    ax.scatter([x_g], [y_g], color='forestgreen', s=140, zorder=5, label='Punto Objetivo $P_{look}$')

    # 10. Acotaciones y Etiquetas de Texto
    ax.text(x1 - 0.6, y1 - 0.5, '$(x_1, y_1)$', fontsize=10, fontweight='bold', color='navy')
    ax.text(x_g + 0.3, y_g + 0.2, '$P_{look} (x_g, y_g)$', fontsize=10, fontweight='bold', color='forestgreen')
    ax.text((x1 + x_g)/2 - 0.2, (y1 + y_g)/2 + 0.4, '$L$', fontsize=11, fontweight='bold', color='blue')
    ax.text(x1 + 1.2, y1 + 0.3, r'$\eta$', fontsize=12, fontweight='bold', color='crimson')

    # Configuración final de ejes y cuadrícula
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('Eje X Global [m]', fontsize=10)
    ax.set_ylabel('Eje Y Global [m]', fontsize=10)
    ax.set_title('Geometría del Algoritmo Pure Pursuit (Marcha Adelante)', fontsize=12, fontweight='bold', pad=12)
    ax.legend(loc='upper left', fontsize=8, framealpha=0.9)

    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f"Imagen guardada exitosamente en: {output_path}")

def generar_diagrama_ray_casting(output_path="./ray_casting_end_of_path.png"):
    """
    Objetivo virtual de fin de ruta tal como lo calcula fin_ruta.punto_objetivo_extendido():
    sobre la prolongación recta del último segmento, ANCLADA EN EL ÚLTIMO PUNTO P_N, a distancia L
    del remolque (no desde el remolque en la dirección del último segmento).
    """
    from arx_truck_ai.fin_ruta import punto_objetivo_extendido
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 5), dpi=300)

    # Waypoints de ruta corregidos
    x_ruta = np.array([0.0, 2.0, 4.0, 6.0, 8.0])
    y_ruta = np.array([0.0, 0.5, 1.2, 1.8, 2.0])

    # Posición del remolque cerca del final, desviada lateralmente de la ruta
    x_tr, y_tr = 7.0, 1.3
    L = 4.0  # Lookahead

    # Vector tangente d_last de la última arista
    dx = x_ruta[-1] - x_ruta[-2]
    dy = y_ruta[-1] - y_ruta[-2]
    norm = np.hypot(dx, dy)
    ux, uy = dx / norm, dy / norm

    # Punto objetivo virtual: el mismo cálculo que usan los controladores
    x_target, y_target = punto_objetivo_extendido(list(zip(x_ruta, y_ruta)), (x_tr, y_tr), L)

    # Graficar ruta
    ax.plot(x_ruta, y_ruta, 'ro-', linewidth=2, markersize=6, label=r'Waypoints de Ruta ($P_k$)')

    # Graficar Ray-Casting (proyección tangente)
    x_proj = np.linspace(x_ruta[-1], x_target + 1.0, 50)
    y_proj = y_ruta[-1] + (uy / ux) * (x_proj - x_ruta[-1])
    ax.plot(x_proj, y_proj, 'b--', linewidth=1.5, label=r'Tangente Proyectada ($\mathbf{d}_{last}$)')

    # Remolque y punto objetivo virtual
    ax.scatter([x_tr], [y_tr], color='navy', s=120, zorder=5, label=r'Eje Remolque $(x_2, y_2)$')
    ax.scatter([x_target], [y_target], color='forestgreen', s=140, zorder=5, label=r'Punto Objetivo Virtual $P_{look}^*$')

    # Cuerda L
    ax.plot([x_tr, x_target], [y_tr, y_target], color='forestgreen', linestyle=':', linewidth=2, label=f'Distancia de Mira $L(v) = {L}$m')

    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('Eje X Global [m]')
    ax.set_ylabel('Eje Y Global [m]')
    ax.set_title('Extrapolación de Fin de Ruta por Ray-Casting (Prevención de Jackknife Terminal)', fontsize=11, fontweight='bold')
    ax.legend(loc='upper left', fontsize=8)

    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f"Imagen guardada en: {output_path}")



# if __name__ == "__main__":
#     main()