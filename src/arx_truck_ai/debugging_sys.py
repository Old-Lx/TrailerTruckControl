import logging
import matplotlib.pyplot as plt

# ==========================================
# MÓDULO DE DEBUGGING Y EVALUACIÓN LQR
# ==========================================
logging.getLogger('beamngpy').setLevel(logging.WARNING)

# Configuración del Logger para generar el archivo de texto
lqr_logger = logging.getLogger('lqr_telemetry')
lqr_logger.setLevel(logging.DEBUG)
file_handler = logging.FileHandler('telemetria_lqr.log', mode='w')
file_handler.setFormatter(logging.Formatter('%(message)s'))
lqr_logger.addHandler(file_handler)
lqr_logger.propagate = False # Evita que nuestros datos se mezclen con la consola global

# Diccionario global para almacenar el historial de datos en memoria para la gráfica
historial_lqr = {
    'tiempo': [],
    'x1_lat': [],
    'x2_ang': [],
    'x3_tij': [],
    'volante': []
}

def registrar_estado(t, x1, x2, x3, volante):
    """
    Guarda los datos en la memoria para graficar y los escribe en el archivo .log
    """
    # Guardar en memoria
    historial_lqr['tiempo'].append(t)
    historial_lqr['x1_lat'].append(x1)
    historial_lqr['x2_ang'].append(x2)
    historial_lqr['x3_tij'].append(x3)
    historial_lqr['volante'].append(volante)
    
    # Escribir en el log
    lqr_logger.debug(f"Lat(x1): {x1:.4f} | Ang(x2): {x2:.4f} | Tij(x3): {x3:.4f} | Volante: {volante:.4f} | Tiempo: {t:.4f}")

def generar_grafico_evaluacion():
    """
    Genera un panel de 3 gráficas (Subplots) usando matplotlib para evaluar 
    la respuesta transitoria del controlador al finalizar la simulación.
    """
    t = historial_lqr['tiempo']
    
    if len(t) == 0:
        print("No hay datos de telemetría para graficar.")
        return

    # Crear una figura con 3 subgráficos apilados
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('Evaluación de Respuesta del Controlador LQR (Marcha Atrás)', fontsize=14)

    # Gráfica 1: Errores de Seguimiento de Trayectoria (x1 y x2)
    ax1.plot(t, historial_lqr['x1_lat'], label='Error Lateral (m)', color='blue')
    ax1.plot(t, historial_lqr['x2_ang'], label='Error Angular (rad)', color='cyan', linestyle='--')
    ax1.set_ylabel('Errores Trayectoria')
    ax1.grid(True)
    ax1.legend(loc='upper right')

    # Gráfica 2: Ángulo de Articulación / Efecto Tijera (x3)
    ax2.plot(t, historial_lqr['x3_tij'], label='Articulación (rad)', color='red')
    ax2.axhline(y=0, color='black', linewidth=1) # Línea cero de referencia
    ax2.set_ylabel('Efecto Tijera')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    # Gráfica 3: Esfuerzo de Control (Volante)
    ax3.plot(t, historial_lqr['volante'], label='Comando Volante [-1, 1]', color='green')
    ax3.axhline(y=1.0, color='gray', linestyle=':') # Límite físico
    ax3.axhline(y=-1.0, color='gray', linestyle=':') # Límite físico
    ax3.set_ylabel('Esfuerzo de Control')
    ax3.set_xlabel('Tiempo de Simulación (s)')
    ax3.grid(True)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('respuesta_sistema_lqr.png') # Guarda la imagen
    print("Gráfica guardada como 'respuesta_sistema_lqr.png'. Abriendo ventana...")
    plt.show() # Muestra la ventana interactiva

# if __name__ == "__main__":
#     main()