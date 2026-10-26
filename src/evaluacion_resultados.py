"""
evaluacion_resultados.py - Capítulo 5: métricas cuantitativas, tablas y figuras comparativas.

Entradas (en la carpeta de datos, por defecto la actual), escritas por los tres algoritmos:
    telemetria_<algoritmo>_<ruta>.csv    una fila por paso de simulación (60 Hz en modo síncrono)
    telemetria_<algoritmo>_<ruta>.json   metadatos de la corrida (resultado, modo, sobrepaso, ...)
    ruta_<ruta>.csv                      ruta de referencia completa (x, y)
  <algoritmo> ∈ {forward, backward, lqr}
  <ruta>      ∈ {recta_x, recta_y, recta_diagonal, parabola_x, parabola_y, seno_x, seno_y}
  (También se aceptan los CSV del formato anterior: telemetria_<algoritmo>_{recta,parabola,seno}.csv.)

Salidas (en la carpeta de salida, por defecto figuras/cap5/):
    metricas.csv, tabla_metricas.md, tabla_metricas.tex
    trayectoria_recta_xy.{png,pdf}, trayectoria_parabola_xy.{png,pdf}, trayectoria_seno_xy.{png,pdf}
    respuestas_temporales_comparativas_<ruta>.{png,pdf}   (una por variante de ruta con datos)

Uso:
    python evaluacion_resultados.py
    python evaluacion_resultados.py --datos corridas/ --salida figuras/cap5 --formatos png pdf --dpi 300
    python evaluacion_resultados.py --unidad-articulacion rad

Métricas (por corrida, sobre todas las muestras registradas del seguimiento):
    RMSE_lat  = sqrt(mean(x1^2))            x1: error lateral con signo del punto de control [m]
                                             (tractor en marcha adelante, remolque en reversa)
    RMSE_yaw  = sqrt(mean(e_trailer^2))     e_trailer: error angular del remolque respecto de la tangente
                                             de la ruta en su punto más cercano [rad]
    theta_max = max|delta_F2|               ángulo máximo de articulación [°]
    S_smooth  = 1/(N-1) * sum|u_{k+1}-u_k|  índice de suavidad del mando de dirección u ∈ [-1, 1]
                                             (adimensional; depende de la frecuencia de registro: compare
                                             corridas registradas a la misma frecuencia)
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd

# =====================================================================================
# CONFIGURACIÓN
# =====================================================================================
# Algoritmos: clave del archivo, nombre para tablas y figuras, color (paleta categorical validada para
# daltonismo, slots 1-3) y estilo de línea (codificación secundaria: legible también impreso en gris).
ALGORITMOS = [
    dict(clave='forward',  nombre='Forward (Pure Pursuit)', corto='Forward',
         color='#2a78d6', estilo='-'),
    dict(clave='backward', nombre='Backward (Aditivo)',     corto='Aditivo',
         color='#eb6834', estilo='--'),
    dict(clave='lqr',      nombre='Backward (LQR)',         corto='LQR',
         color='#1baf7a', estilo='-.'),
]

# Familias de trayectorias y sus variantes (en el orden de las figuras y tablas)
FAMILIAS = {
    'recta':    dict(titulo='tramo rectilíneo',  rutas=['recta_x', 'recta_y', 'recta_diagonal']),
    'parabola': dict(titulo='parábola',          rutas=['parabola_x', 'parabola_y']),
    'seno':     dict(titulo='senoide en "S"',    rutas=['seno_x', 'seno_y']),
}
ETIQUETA_RUTA = {          # figuras (mathtext de Matplotlib)
    'recta_x':        'Recta paralela a X',
    'recta_y':        'Recta paralela a Y',
    'recta_diagonal': 'Recta $x = y$',
    'parabola_x':     'Parábola $y = a\\,x^2$',
    'parabola_y':     'Parábola $x = a\\,y^2$',
    'seno_x':         'Senoide $y = A\\,\\sin(\\omega x)$',
    'seno_y':         'Senoide $x = A\\,\\sin(\\omega y)$',
}
ETIQUETA_RUTA_TXT = {      # Markdown / consola
    'recta_x': 'Recta ∥ X', 'recta_y': 'Recta ∥ Y', 'recta_diagonal': 'Recta x = y',
    'parabola_x': 'Parábola y = a·x²', 'parabola_y': 'Parábola x = a·y²',
    'seno_x': 'Senoide y = A·sin(ωx)', 'seno_y': 'Senoide x = A·sin(ωy)',
}
ETIQUETA_RUTA_TEX = {      # LaTeX
    'recta_x': 'Recta $\\parallel X$', 'recta_y': 'Recta $\\parallel Y$', 'recta_diagonal': 'Recta $x = y$',
    'parabola_x': 'Parábola $y = a x^2$', 'parabola_y': 'Parábola $x = a y^2$',
    'seno_x': 'Senoide $y = A\\sin(\\omega x)$', 'seno_y': 'Senoide $x = A\\sin(\\omega y)$',
}
ALIAS_LEGADO = {'recta_x': 'recta', 'parabola_x': 'parabola', 'seno_x': 'seno'}   # nombres del formato anterior

RESULTADO_TXT = {'completada': 'Completada', 'jackknife': 'Jackknife', 'tiempo_maximo': 'Tiempo máximo',
                 'desconocido': '—'}

# Líneas de referencia de la articulación [rad] (umbrales del sistema)
UMBRAL_PANICO_RAD = 0.42      # límite duro de pánico del controlador aditivo
UMBRAL_JACKKNIFE_RAD = 1.0    # corte por jackknife en backward.py / backwards_lqr.py

# Estilo (figuras de tesis)
TINTA = '#0b0b0b'             # texto principal
TINTA_2 = '#52514e'           # texto secundario / ejes
GUIA = '#d9d8d3'              # rejilla y líneas de referencia
RUTA_COLOR = '#b9b8b2'        # banda de la ruta de referencia
SUPERFICIE = '#ffffff'

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 10, 'axes.titlesize': 10, 'axes.labelsize': 10,
    'legend.fontsize': 9, 'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'axes.edgecolor': TINTA_2, 'axes.labelcolor': TINTA, 'xtick.color': TINTA_2, 'ytick.color': TINTA_2,
    'axes.grid': True, 'grid.color': GUIA, 'grid.linewidth': 0.6, 'axes.axisbelow': True,
    'axes.spines.top': False, 'axes.spines.right': False,
    'figure.facecolor': SUPERFICIE, 'axes.facecolor': SUPERFICIE, 'savefig.facecolor': SUPERFICIE,
    'lines.linewidth': 1.5, 'mathtext.fontset': 'dejavuserif',
})


# =====================================================================================
# CARGA DE DATOS
# =====================================================================================
class Corrida:
    """Una corrida (algoritmo × ruta): telemetría, metadatos y diagnóstico de formato."""

    def __init__(self, algoritmo, ruta, df, meta, archivo, legado):
        self.algoritmo, self.ruta, self.df, self.meta, self.archivo, self.legado = algoritmo, ruta, df, meta, archivo, legado

    @property
    def resultado(self):
        r = self.meta.get('resultado')
        if r:
            return r
        d = self.df['delta_F2'].to_numpy()
        return 'jackknife' if len(d) and abs(d[-1]) >= 0.95 * UMBRAL_JACKKNIFE_RAD else 'desconocido'

    @property
    def frecuencia_hz(self):
        t = self.df['t'].to_numpy()
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float('nan')

    def xy_control(self):
        return self.df['x_real'].to_numpy(), self.df['y_real'].to_numpy()


def _leer_json(ruta_json):
    try:
        with open(ruta_json, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def cargar_corrida(carpeta, algoritmo, ruta):
    """Carga telemetria_<algoritmo>_<ruta>.csv (o su nombre del formato anterior). None si no existe."""
    candidatos = [ruta] + ([ALIAS_LEGADO[ruta]] if ruta in ALIAS_LEGADO else [])
    for nombre in candidatos:
        archivo = os.path.join(carpeta, f"telemetria_{algoritmo}_{nombre}.csv")
        if not os.path.isfile(archivo):
            continue
        df = pd.read_csv(archivo)
        if df.empty or 't' not in df.columns:
            print(f"  aviso: {os.path.basename(archivo)} está vacío o sin columna t; se omite")
            return None
        df = df.dropna(subset=['t']).sort_values('t').reset_index(drop=True)
        legado = ('x_tractor' not in df.columns) or df['x_tractor'].isna().all()
        meta = _leer_json(os.path.splitext(archivo)[0] + '.json')
        return Corrida(algoritmo, ruta, df, meta, archivo, legado)
    return None


def cargar_ruta_referencia(carpeta, ruta, corridas):
    """Ruta de referencia completa (ruta_<ruta>.csv); si no existe, la reconstruye con x_ref/y_ref."""
    archivo = os.path.join(carpeta, f"ruta_{ruta}.csv")
    if os.path.isfile(archivo):
        r = pd.read_csv(archivo)
        if {'x', 'y'} <= set(r.columns) and len(r) > 1:
            return r['x'].to_numpy(), r['y'].to_numpy(), True
    for c in corridas:
        if c is not None and {'x_ref', 'y_ref'} <= set(c.df.columns):
            ref = c.df[['x_ref', 'y_ref']].dropna()
            if len(ref) > 1:
                return ref['x_ref'].to_numpy(), ref['y_ref'].to_numpy(), False
    return None, None, False


# =====================================================================================
# MÉTRICAS
# =====================================================================================
def rmse(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float('nan')


def indice_suavidad(u):
    """S_smooth = 1/(N-1) * sum_k |u_{k+1} - u_k|."""
    u = np.asarray(u, dtype=float)
    u = u[np.isfinite(u)]
    return float(np.mean(np.abs(np.diff(u)))) if len(u) > 1 else float('nan')


def calcular_metricas(c):
    df = c.df
    return dict(
        algoritmo=c.algoritmo, ruta=c.ruta,
        rmse_lat=rmse(df['error_lateral']),
        rmse_yaw=rmse(df['error_angular']),
        theta_max_deg=float(np.nanmax(np.abs(df['delta_F2'].to_numpy())) * 180.0 / math.pi),
        s_smooth=indice_suavidad(df['steering']),
        resultado=c.resultado,
        duracion_s=float(df['t'].iloc[-1] - df['t'].iloc[0]),
        muestras=int(len(df)),
        frecuencia_hz=float(c.frecuencia_hz),
        sobrepaso_m=c.meta.get('sobrepaso_m'),
        formato='anterior' if c.legado else 'actual',
        archivo=os.path.basename(c.archivo),
    )


# =====================================================================================
# TABLAS
# =====================================================================================
COLUMNAS_METRICAS = [('rmse_lat', '{:.3f}'), ('rmse_yaw', '{:.3f}'), ('theta_max_deg', '{:.2f}'), ('s_smooth', '{:.4f}')]


def _mejores(filas):
    """Para cada ruta y métrica, el mínimo entre las corridas COMPLETADAS (una corrida que terminó en
    jackknife puede tener errores bajos solo por haber durado menos)."""
    mejores = {}
    for ruta in {f['ruta'] for f in filas}:
        ok = [f for f in filas if f['ruta'] == ruta and f['resultado'] == 'completada']
        for col, _ in COLUMNAS_METRICAS:
            vals = [f[col] for f in ok if np.isfinite(f[col])]
            if vals:
                mejores[(ruta, col)] = min(vals)
    return mejores


def _nombre_alg(clave, campo='nombre'):
    return next((a[campo] for a in ALGORITMOS if a['clave'] == clave), clave)


def tabla_markdown(filas):
    mejores = _mejores(filas)
    lineas = ["| Trayectoria | Algoritmo | RMSE_lat [m] | RMSE_yaw [rad] | θ_max [°] | S_smooth [–] | Resultado | Duración [s] |",
              "|---|---|---:|---:|---:|---:|---|---:|"]
    ruta_prev = None
    for f in filas:
        celdas = []
        for col, fmt in COLUMNAS_METRICAS:
            v = f[col]
            txt = fmt.format(v) if np.isfinite(v) else '—'
            if (f['resultado'] == 'completada' and (f['ruta'], col) in mejores
                    and fmt.format(mejores[(f['ruta'], col)]) == txt):
                txt = f"**{txt}**"
            celdas.append(txt)
        tray = ETIQUETA_RUTA_TXT.get(f['ruta'], f['ruta']) if f['ruta'] != ruta_prev else ''
        ruta_prev = f['ruta']
        lineas.append(f"| {tray} | {_nombre_alg(f['algoritmo'])} | " + " | ".join(celdas)
                      + f" | {RESULTADO_TXT.get(f['resultado'], f['resultado'])} | {f['duracion_s']:.1f} |")
    notas = ["", "En negrita, el mejor valor de cada trayectoria entre las corridas completadas.",
             "RMSE_lat: punto de control (tractor en marcha adelante, remolque en reversa). "
             "RMSE_yaw: error angular del remolque respecto de la tangente de la ruta. "
             "S_smooth = 1/(N−1)·Σ|u_{k+1} − u_k|."]
    if any(f['formato'] == 'anterior' for f in filas):
        notas.append("Las corridas en formato anterior usan el error angular propio de cada controlador; "
                     "repítalas con la telemetría actual para una comparación homogénea.")
    frecs = sorted({round(f['frecuencia_hz']) for f in filas if np.isfinite(f['frecuencia_hz'])})
    if len(frecs) > 1:
        notas.append(f"Atención: las corridas tienen distintas frecuencias de registro ({', '.join(map(str, frecs))} Hz); "
                     "S_smooth solo es comparable entre corridas registradas a la misma frecuencia.")
    return "\n".join(lineas + notas) + "\n"


def tabla_latex(filas):
    mejores = _mejores(filas)
    out = [r"% Requiere \usepackage{booktabs} en el preámbulo",
           r"\begin{table}[htbp]", r"\centering", r"\small",
           r"\caption{Métricas cuantitativas de desempeño por trayectoria y algoritmo.}",
           r"\label{tab:metricas_cap5}",
           r"\begin{tabular}{llrrrrl}", r"\toprule",
           r"Trayectoria & Algoritmo & $RMSE_{lat}$ [m] & $RMSE_{yaw}$ [rad] & $\theta_{max}$ [$^\circ$] & $S_{smooth}$ & Resultado \\",
           r"\midrule"]
    rutas_orden = []
    for f in filas:
        if f['ruta'] not in rutas_orden:
            rutas_orden.append(f['ruta'])
    for i, ruta in enumerate(rutas_orden):
        grupo = [f for f in filas if f['ruta'] == ruta]
        for j, f in enumerate(grupo):
            celdas = []
            for col, fmt in COLUMNAS_METRICAS:
                v = f[col]
                txt = fmt.format(v) if np.isfinite(v) else '--'
                if (f['resultado'] == 'completada' and (ruta, col) in mejores
                        and fmt.format(mejores[(ruta, col)]) == txt):
                    txt = r"\textbf{" + txt + "}"
                celdas.append(txt)
            tray = ETIQUETA_RUTA_TEX.get(ruta, ruta) if j == 0 else ''
            out.append(f"{tray} & {_nombre_alg(f['algoritmo'])} & " + " & ".join(celdas)
                       + f" & {RESULTADO_TXT.get(f['resultado'], f['resultado'])} \\\\")
        if i < len(rutas_orden) - 1:
            out.append(r"\midrule")
    out += [r"\bottomrule", r"\end{tabular}",
            r"\par\smallskip\footnotesize En negrita, el mejor valor de cada trayectoria entre las corridas completadas. "
            r"$S_{smooth} = \frac{1}{N-1}\sum_k |u_{k+1}-u_k|$.",
            r"\end{table}"]
    return "\n".join(out) + "\n"


# =====================================================================================
# FIGURAS
# =====================================================================================
def _guardar(fig, salida, nombre, formatos, dpi):
    rutas = []
    for ext in formatos:
        p = os.path.join(salida, f"{nombre}.{ext}")
        fig.savefig(p, dpi=dpi, bbox_inches='tight')
        rutas.append(p)
    plt.close(fig)
    return rutas


def _manejadores_leyenda(algoritmos_presentes, hay_jackknife, con_ruta=True):
    h = []
    if con_ruta:
        h.append(Line2D([], [], color=RUTA_COLOR, lw=5, solid_capstyle='round', label='Ruta de referencia'))
    for a in ALGORITMOS:
        if a['clave'] in algoritmos_presentes:
            h.append(Line2D([], [], color=a['color'], ls=a['estilo'], lw=1.6, label=a['nombre']))
    if con_ruta:
        h.append(Line2D([], [], color=TINTA_2, marker='o', ls='none', ms=6, mfc='white', label='Inicio de la ruta'))
        h.append(Line2D([], [], color=TINTA_2, marker='s', ls='none', ms=6, label='Fin de la ruta'))
    if hay_jackknife:
        h.append(Line2D([], [], color=TINTA, marker='X', ls='none', ms=8, label='Fin por jackknife'))
    return h


def figura_trayectorias(familia, carpeta, salida, formatos, dpi, max_aspecto_isotropo=3.0):
    """Plano XY: una columna por variante de la familia; punto de control de cada algoritmo sobre la ruta."""
    info = FAMILIAS[familia]
    rutas = info['rutas']
    datos = {ruta: {a['clave']: cargar_corrida(carpeta, a['clave'], ruta) for a in ALGORITMOS} for ruta in rutas}
    if not any(c is not None for d in datos.values() for c in d.values()):
        print(f"  {familia}: sin telemetría; se omite trayectoria_{familia}_xy")
        return []

    n = len(rutas)
    fig, axes = plt.subplots(1, n, figsize=(3.3 * n, 3.9), squeeze=False)
    algoritmos_presentes, hay_jk = set(), False
    for ax, ruta in zip(axes[0], rutas):
        corr = datos[ruta]
        xr, yr, completa = cargar_ruta_referencia(carpeta, ruta, corr.values())
        xs, ys = [], []
        if xr is not None:
            ax.plot(xr, yr, color=RUTA_COLOR, lw=5, solid_capstyle='round', zorder=1)
            ax.plot(xr[0], yr[0], 'o', ms=6, mfc='white', mec=TINTA_2, mew=1.2, zorder=4)
            ax.plot(xr[-1], yr[-1], 's', ms=6, color=TINTA_2, zorder=4)
            xs.append(xr); ys.append(yr)
        vacias = []
        for a in ALGORITMOS:
            c = corr[a['clave']]
            if c is None:
                vacias.append(a['corto'])
                continue
            algoritmos_presentes.add(a['clave'])
            x, y = c.xy_control()
            ax.plot(x, y, color=a['color'], ls=a['estilo'], lw=1.5, zorder=3)
            xs.append(x); ys.append(y)
            if c.resultado == 'jackknife':
                hay_jk = True
                ax.plot(x[-1], y[-1], 'X', ms=8, color=a['color'], mec=TINTA, mew=0.8, zorder=5)
        ax.set_title(ETIQUETA_RUTA[ruta], color=TINTA)
        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        if xs:
            X, Y = np.concatenate(xs), np.concatenate(ys)
            ancho, alto = np.ptp(X[np.isfinite(X)]), np.ptp(Y[np.isfinite(Y)])
            razon = max(ancho, alto) / max(min(ancho, alto), 1e-6)
            ax.ticklabel_format(useOffset=False, style='plain')
            if razon <= max_aspecto_isotropo:
                ax.set_aspect('equal', adjustable='datalim')
            else:
                # Trayectoria muy alargada (recta, senoide de baja amplitud): escala transversal ampliada,
                # con un rango mínimo para no amplificar el ruido numérico (p. ej. desvíos de 1e-13 m)
                largo = max(ancho, alto)
                minimo = max(min(ancho, alto) * 1.15, largo / 20.0, 4.0)
                X_, Y_ = X[np.isfinite(X)], Y[np.isfinite(Y)]
                if ancho >= alto:
                    c = 0.5 * (Y_.min() + Y_.max()); ax.set_ylim(c - minimo / 2, c + minimo / 2)
                    ax.set_ylabel('$y$ [m] (escala ampliada)')
                else:
                    c = 0.5 * (X_.min() + X_.max()); ax.set_xlim(c - minimo / 2, c + minimo / 2)
                    ax.set_xlabel('$x$ [m] (escala ampliada)')
        if vacias:
            ax.text(0.98, 0.98, 'sin datos: ' + ', '.join(vacias), transform=ax.transAxes,
                    fontsize=7.5, color=TINTA_2, ha='right', va='top')
        if xr is not None and not completa:
            ax.text(0.02, 0.98, 'ruta reconstruida (sin ruta_*.csv)', transform=ax.transAxes,
                    fontsize=7.5, color=TINTA_2, va='top')
    fig.suptitle(f"Seguimiento de trayectoria en el plano XY: {info['titulo']}", color=TINTA)
    fig.tight_layout(rect=(0, 0.13, 1, 1))
    fig.legend(handles=_manejadores_leyenda(algoritmos_presentes, hay_jk), loc='lower center',
               bbox_to_anchor=(0.5, 0.0), ncol=4, frameon=False)
    return _guardar(fig, salida, f"trayectoria_{familia}_xy", formatos, dpi)


def figura_temporal(ruta, carpeta, salida, formatos, dpi, unidad='deg'):
    """Subplots 3x1: error lateral x1(t), articulación delta_F2(t) y comando de dirección u(t)."""
    corr = {a['clave']: cargar_corrida(carpeta, a['clave'], ruta) for a in ALGORITMOS}
    if all(c is None for c in corr.values()):
        return []
    k = 180.0 / math.pi if unidad == 'deg' else 1.0
    u_txt = '[°]' if unidad == 'deg' else '[rad]'
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7.0, 6.6), sharex=True)
    presentes, hay_jk = set(), False
    for a in ALGORITMOS:
        c = corr[a['clave']]
        if c is None:
            continue
        presentes.add(a['clave'])
        df = c.df
        t = df['t'].to_numpy() - df['t'].iloc[0]
        kw = dict(color=a['color'], ls=a['estilo'], lw=1.4)
        ax1.plot(t, df['error_lateral'].to_numpy(), **kw)
        ax2.plot(t, df['delta_F2'].to_numpy() * k, **kw)
        ax3.plot(t, df['steering'].to_numpy(), **kw)
        if c.resultado == 'jackknife':
            hay_jk = True
            ax2.plot(t[-1], df['delta_F2'].iloc[-1] * k, 'X', ms=8, color=a['color'], mec=TINTA, mew=0.8, zorder=5)

    ax1.axhline(0.0, color=TINTA_2, lw=0.8, zorder=1)
    ax1.set_ylabel('Error lateral $x_1$ [m]')
    ax2.axhline(0.0, color=TINTA_2, lw=0.8, zorder=1)
    for valor, texto in ((UMBRAL_PANICO_RAD, 'umbral de pánico'), (UMBRAL_JACKKNIFE_RAD, 'corte por jackknife')):
        for signo in (1, -1):
            ax2.axhline(signo * valor * k, color=TINTA_2, lw=0.8, ls=':', zorder=1)
        ax2.annotate(f"{texto} (±{valor:.2f} rad)", xy=(1.0, valor * k), xycoords=('axes fraction', 'data'),
                     xytext=(-4, 3), textcoords='offset points', ha='right', va='bottom', fontsize=7.5, color=TINTA_2)
    lim = max(UMBRAL_JACKKNIFE_RAD * k * 1.08, ax2.get_ylim()[1], -ax2.get_ylim()[0])
    ax2.set_ylim(-lim, lim)
    ax2.set_ylabel(f'Articulación $\\delta_{{F2}}$ {u_txt}')
    for s in (1.0, -1.0):
        ax3.axhline(s, color=TINTA_2, lw=0.8, ls=':', zorder=1)
    ax3.axhline(0.0, color=TINTA_2, lw=0.8, zorder=1)
    ax3.set_ylim(-1.1, 1.1)
    ax3.set_ylabel('Dirección $u \\in [-1, 1]$')
    ax3.set_xlabel('Tiempo $t$ [s]')
    fig.suptitle(f"Respuesta temporal comparativa: {ETIQUETA_RUTA[ruta]}", color=TINTA)
    fig.align_ylabels((ax1, ax2, ax3))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.legend(handles=_manejadores_leyenda(presentes, hay_jk, con_ruta=False), loc='lower center',
               bbox_to_anchor=(0.5, 0.0), ncol=4, frameon=False)
    return _guardar(fig, salida, f"respuestas_temporales_comparativas_{ruta}", formatos, dpi)


# =====================================================================================
# PROGRAMA PRINCIPAL
# =====================================================================================
def evaluar(carpeta='.', salida='figuras/cap5', formatos=('png', 'pdf'), dpi=300, unidad='deg'):
    os.makedirs(salida, exist_ok=True)
    filas = []
    print(f"Leyendo telemetría de: {os.path.abspath(carpeta)}")
    for familia, info in FAMILIAS.items():
        for ruta in info['rutas']:
            for a in ALGORITMOS:
                c = cargar_corrida(carpeta, a['clave'], ruta)
                if c is None:
                    continue
                m = calcular_metricas(c)
                filas.append(m)
                print(f"  {m['archivo']:42s} {m['muestras']:6d} muestras @ {m['frecuencia_hz']:5.1f} Hz | "
                      f"{RESULTADO_TXT.get(m['resultado'], m['resultado'])}{' | formato anterior' if c.legado else ''}")
    if not filas:
        print("No se encontró ningún archivo telemetria_<algoritmo>_<ruta>.csv.")
        return 1

    df_m = pd.DataFrame(filas)
    df_m.to_csv(os.path.join(salida, 'metricas.csv'), index=False)
    md, tex = tabla_markdown(filas), tabla_latex(filas)
    with open(os.path.join(salida, 'tabla_metricas.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    with open(os.path.join(salida, 'tabla_metricas.tex'), 'w', encoding='utf-8') as f:
        f.write(tex)

    generadas = []
    for familia in FAMILIAS:
        generadas += figura_trayectorias(familia, carpeta, salida, formatos, dpi)
        for ruta in FAMILIAS[familia]['rutas']:
            generadas += figura_temporal(ruta, carpeta, salida, formatos, dpi, unidad)

    print("\n" + "=" * 100)
    print("TABLA CONSOLIDADA DE MÉTRICAS CUANTITATIVAS")
    print("=" * 100)
    print(md)
    print(f"Archivos escritos en {os.path.abspath(salida)}:")
    for p in ['metricas.csv', 'tabla_metricas.md', 'tabla_metricas.tex'] + [os.path.basename(g) for g in generadas]:
        print(f"  {p}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Métricas, tablas y figuras del Capítulo 5.")
    ap.add_argument('--datos', default='.', help="carpeta con telemetria_*.csv (por defecto, la actual)")
    ap.add_argument('--salida', default='figuras/cap5', help="carpeta de salida (por defecto figuras/cap5)")
    ap.add_argument('--formatos', nargs='+', default=['png', 'pdf'], help="formatos de figura (png pdf svg ...)")
    ap.add_argument('--dpi', type=int, default=300)
    ap.add_argument('--unidad-articulacion', choices=['deg', 'rad'], default='deg')
    args = ap.parse_args(argv)
    return evaluar(args.datos, args.salida, args.formatos, args.dpi, args.unidad_articulacion)


if __name__ == "__main__":
    sys.exit(main())
