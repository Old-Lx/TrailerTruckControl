import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# Configuración de estilo para tesis (formato académico)
plt.rcParams.update({'font.size': 12, 'font.family': 'serif'})
DIRECTORIO_SALIDA = "figuras/cap5/"
os.makedirs(DIRECTORIO_SALIDA, exist_ok=True)

def cargar_datos(trayectoria):
    """Carga los CSVs de los 3 algoritmos para una trayectoria específica."""
    try:
        df_fwd = pd.read_csv(f"telemetria_forward_{trayectoria}.csv")
        df_bwd = pd.read_csv(f"telemetria_backward_{trayectoria}.csv")
        df_lqr = pd.read_csv(f"telemetria_lqr_{trayectoria}.csv")
        return df_fwd, df_bwd, df_lqr
    except FileNotFoundError as e:
        print(f"Falta archivo CSV: {e}")
        return None, None, None

def calcular_metricas(df, nombre_algo):
    """Calcula las métricas cuantitativas para la Tabla Consolidada 5.5."""
    rmse_lat = np.sqrt(np.mean(df['error_lateral']**2))
    rmse_yaw = np.sqrt(np.mean(df['error_angular']**2))
    max_tijera = np.max(np.abs(df['delta_F2'])) * (180.0 / np.pi) # En grados
    
    # S_smooth: integral de la derivada del control (penaliza chattering)
    derivada_u = np.diff(df['steering']) / np.diff(df['t'])
    s_smooth = np.sqrt(np.mean(derivada_u**2))
    
    return [nombre_algo, f"{rmse_lat:.3f}", f"{rmse_yaw:.3f}", f"{max_tijera:.2f}", f"{s_smooth:.2f}"]

def generar_grafico_xy(df_fwd, df_bwd, df_lqr, trayectoria, numero_figura):
    """Genera gráficas 2D en el plano XY (Punto 2 de la lista)."""
    plt.figure(figsize=(10, 6))
    
    # Se asume que la referencia es igual para todos en esa trayectoria
    plt.plot(df_fwd['x_ref'], df_fwd['y_ref'], 'k--', label='Ruta Referencia', linewidth=2)
    plt.plot(df_fwd['x_real'], df_fwd['y_real'], 'g-', label='Forward', alpha=0.8)
    plt.plot(df_bwd['x_real'], df_bwd['y_real'], 'b-', label='Backward (Aditivo)', alpha=0.8)
    plt.plot(df_lqr['x_real'], df_lqr['y_real'], 'r-', label='Backward (LQR)', alpha=0.8)
    
    plt.title(f'Seguimiento de Trayectoria: {trayectoria.capitalize()}')
    plt.xlabel('Coordenada X [m]')
    plt.ylabel('Coordenada Y [m]')
    plt.legend()
    plt.grid(True, linestyle=':')
    plt.axis('equal')
    
    plt.tight_layout()
    plt.savefig(f"{DIRECTORIO_SALIDA}/{numero_figura}_trayectoria_{trayectoria}_xy.png", dpi=300)
    plt.close()

def generar_comparativa_temporal(df_fwd, df_bwd, df_lqr, trayectoria):
    """Genera los 3 subplots temporales (Punto 3 de la lista)."""
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    algoritmos = [('Forward', df_fwd, 'g'), ('Backward (Aditivo)', df_bwd, 'b'), ('Backward (LQR)', df_lqr, 'r')]
    
    for nombre, df, color in algoritmos:
        axs[0].plot(df['t'], df['error_lateral'], color=color, label=nombre)
        axs[1].plot(df['t'], df['delta_F2'], color=color, label=nombre)
        axs[2].plot(df['t'], df['steering'], color=color, label=nombre)

    axs[0].set_ylabel('Error Lateral $x_1$ [m]')
    axs[0].set_title(f'Análisis Temporal Comparativo - {trayectoria.capitalize()}')
    axs[0].grid(True)
    axs[0].legend(loc='upper right')

    axs[1].set_ylabel(r'Articulación $\delta_{F2}$ [rad]')
    axs[1].axhline(y=0.45, color='k', linestyle='--', alpha=0.5) # Límite tijera
    axs[1].axhline(y=-0.45, color='k', linestyle='--', alpha=0.5)
    axs[1].grid(True)

    axs[2].set_ylabel('Comando Dirección $u \in [-1, 1]$')
    axs[2].set_xlabel('Tiempo $t$ [s]')
    axs[2].set_ylim([-1.1, 1.1])
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig(f"{DIRECTORIO_SALIDA}/4_respuestas_temporales_comparativas_{trayectoria}.png", dpi=300)
    plt.close()

def main():
    trayectorias = [('recta', 1), ('parabola', 2), ('seno', 3)]
    tabla_resultados = []
    
    for tray, num_fig in trayectorias:
        print(f"\\nProcesando trayectoria: {tray}...")
        df_fwd, df_bwd, df_lqr = cargar_datos(tray)
        
        if df_fwd is not None:
            generar_grafico_xy(df_fwd, df_bwd, df_lqr, tray, num_fig)
            generar_comparativa_temporal(df_fwd, df_bwd, df_lqr, tray)
            
            tabla_resultados.append(calcular_metricas(df_fwd, f"Forward ({tray})"))
            tabla_resultados.append(calcular_metricas(df_bwd, f"Backward ({tray})"))
            tabla_resultados.append(calcular_metricas(df_lqr, f"LQR ({tray})"))
    
    # Imprimir Tabla 5.5 Consolidada
    if tabla_resultados:
        print("\\n" + "="*80)
        print("TABLA CONSOLIDADA DE MÉTRICAS CUANTITATIVAS (Sección 5.5)")
        print("="*80)
        print(f"{'Algoritmo (Trayectoria)':<25} | {'RMSE_lat [m]':<12} | {'RMSE_yaw [rad]':<14} | {'Theta_max [°]':<13} | {'S_smooth':<10}")
        print("-" * 80)
        for fila in tabla_resultados:
            print(f"{fila[0]:<25} | {fila[1]:<12} | {fila[2]:<14} | {fila[3]:<13} | {fila[4]:<10}")
        print("="*80)

if __name__ == "__main__":
    main()
