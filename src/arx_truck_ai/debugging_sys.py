import logging
import matplotlib.pyplot as plt

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

def registrar_estado(t, error_lateral, error_angular, articulacion, volante):
    """
    Guarda los datos en la memoria para graficar y los escribe en el archivo .log
    """
    # Guardar en memoria
    historial['tiempo'].append(t)
    historial['x1_lat'].append(error_lateral)
    historial['x2_ang'].append(error_angular)
    historial['x3_tij'].append(articulacion)
    historial['volante'].append(volante)
    
    # Escribir en el log
    logger.debug(f"Lat(x1): {error_lateral:.4f} | Ang(x2): {error_angular:.4f} | Tij(x3): {articulacion:.4f} | Volante: {volante:.4f} | Tiempo: {t:.4f}")

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

# if __name__ == "__main__":
#     main()