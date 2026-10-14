import csv
import logging
import matplotlib.pyplot as plt
import numpy as np
import os

# ==========================================
# MÓDULO DE DEBUGGING Y EVALUACIÓN
# ==========================================
logging.getLogger('beamngpy').setLevel(logging.WARNING)

# Configuración del Logger para generar el archivo de texto
logger = logging.getLogger('telemetry')
logger.setLevel(logging.DEBUG)
file_handler = logging.FileHandler('telemetria_sistema.log', mode='w')
file_handler.setFormatter(logging.Formatter('%(message)s'))
logger.addHandler(file_handler)
logger.propagate = False # Evita que nuestros datos se mezclen con la consola global

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
# Nombre del CSV activo. Lo fija inicializar_telemetria(); mientras sea None,
# registrar_estado() solo escribe en memoria y en el .log.
archivo_csv_activo = None

# Cabecera EXACTA que espera el script de evaluación.
CSV_CABECERA = ['t', 'x_real', 'y_real', 'x_ref', 'y_ref',
                'error_lateral', 'error_angular', 'delta_F2', 'steering']

_aviso_sin_csv_emitido = False  # avisar UNA vez, no en cada tick

def inicializar_telemetria(nombre_archivo):
    """
    Crea (o sobrescribe) el CSV `nombre_archivo` con la cabecera estándar y lo
    deja como archivo activo para registrar_estado(). Llamar UNA vez por
    experimento, antes del bucle de control.
    """
    global archivo_csv_activo, _aviso_sin_csv_emitido
    archivo_csv_activo = nombre_archivo
    _aviso_sin_csv_emitido = False
    with open(archivo_csv_activo, 'w', newline='', encoding='utf-8') as f:
        csv.writer(f, lineterminator='\n').writerow(CSV_CABECERA)

def registrar_estado(t, x_real, y_real, x_ref, y_ref, error_lateral, error_angular, articulacion, volante):
    """
    Guarda los datos en la memoria para graficar, los escribe en el archivo .log
    y añade una fila al CSV activo (ver inicializar_telemetria).
    """
    global _aviso_sin_csv_emitido

    # Guardar en memoria
    historial['tiempo'].append(t)
    historial['x1_lat'].append(error_lateral)
    historial['x2_ang'].append(error_angular)
    historial['x3_tij'].append(articulacion)
    historial['volante'].append(volante)
    
    # Escribir en el log
    logger.debug(f"Lat(x1): {error_lateral:.4f} | Ang(x2): {error_angular:.4f} | Tij(x3): {articulacion:.4f} | Volante: {volante:.4f} | Tiempo: {t:.4f}")

    # Escribir en el CSV (modo append: cada fila queda en disco al instante, así
    # los datos sobreviven aunque la simulación termine en jackknife o se corte)
    if archivo_csv_activo is not None:
        with open(archivo_csv_activo, 'a', newline='', encoding='utf-8') as f:
            csv.writer(f, lineterminator='\n').writerow(
                [t, x_real, y_real, x_ref, y_ref,
                 error_lateral, error_angular, articulacion, volante]
            )
    elif not _aviso_sin_csv_emitido:
        print("[telemetria] AVISO: registrar_estado() se llamó sin inicializar_telemetria(); "
              "no se está escribiendo ningún CSV.")
        _aviso_sin_csv_emitido = True

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

# Para backward
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
            color='navy', linewidth=2, label='Eje Longitudinal ($\psi_1$)')

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
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 5), dpi=300)

    # Waypoints de ruta corregidos
    x_ruta = np.array([0.0, 2.0, 4.0, 6.0, 8.0])
    y_ruta = np.array([0.0, 0.5, 1.2, 1.8, 2.0])

    # Posición del remolque cerca del final
    x_tr, y_tr = 7.0, 1.9
    L = 4.0  # Lookahead

    # Vector tangente d_last de la última arista
    dx = x_ruta[-1] - x_ruta[-2]
    dy = y_ruta[-1] - y_ruta[-2]
    norm = np.hypot(dx, dy)
    ux, uy = dx / norm, dy / norm

    # Punto objetivo proyectado por Ray-Casting
    x_target = x_tr + ux * L
    y_target = y_tr + uy * L

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