"""
metricas_complementarias.py - Métricas del Capítulo 5 que no calcula evaluacion_resultados.py.

Por corrida (una carpeta por repetición, mismos nombres de archivo que la campaña):
  - sobrepaso_captura_m: tras el primer cruce por cero del error lateral x1, el mayor |x1| con signo opuesto
    al del error inicial (cuánto se pasa de la ruta al capturarla). Vacío si |x1| nunca supera 0.5 m.
  - saturacion_pct: porcentaje de muestras con el volante saturado (|u| > 0.99).
  - amplitud_relativa: solo en las senoides; semiamplitud de la trayectoria del punto de control en el tramo
    central (60-200 m en seno_x, 60-190 m en seno_y) dividida entre la amplitud de la ruta, A = 3 m.
    < 1: el controlador recorta la curva; > 1: la amplifica.
  - velocidad_media_ms: media de |v|.

Uso (desde la raíz del repositorio):
    python src/metricas_complementarias.py --datos corridas/campana_r1 corridas/campana_r2 corridas/campana_r3 \
        --salida corridas/campana_eval
Escribe metricas_complementarias.csv (por corrida) y metricas_complementarias_agregadas.csv (media y desviación
estándar muestral por caso).
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

ALGORITMOS = ('forward', 'backward', 'lqr')
RUTAS = ('recta_x', 'recta_y', 'recta_diagonal', 'parabola_x', 'parabola_y', 'seno_x', 'seno_y')
AMPLITUD_SENO_M = 3.0
TRAMO_SENO = {'seno_x': ('x_real', 'y_real', 60.0, 200.0), 'seno_y': ('y_real', 'x_real', 60.0, 190.0)}
UMBRAL_ERROR_INICIAL_M = 0.5
UMBRAL_SATURACION = 0.99


def metricas(df, ruta):
    e = df['error_lateral'].to_numpy(dtype=float)       # misma definición que evaluacion_resultados.py
    u = df['steering'].to_numpy(dtype=float)
    d = dict(sobrepaso_captura_m=float('nan'), amplitud_relativa=float('nan'))
    grandes = np.abs(e) > UMBRAL_ERROR_INICIAL_M
    if np.any(grandes):
        s0 = np.sign(e[np.argmax(grandes)])
        cruces = np.where(np.sign(e) == -s0)[0]
        d['sobrepaso_captura_m'] = float(np.max(-s0 * e[cruces[0]:])) if len(cruces) else 0.0
    d['saturacion_pct'] = 100.0 * float(np.mean(np.abs(u) > UMBRAL_SATURACION))
    if ruta in TRAMO_SENO:
        eje, lateral, a, b = TRAMO_SENO[ruta]
        m = (df[eje] > a) & (df[eje] < b)
        d['amplitud_relativa'] = float((df[lateral][m].max() - df[lateral][m].min()) / 2.0 / AMPLITUD_SENO_M)
    d['velocidad_media_ms'] = float(df['velocidad'].abs().mean())
    return d


def main(argv=None):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    ap = argparse.ArgumentParser(description="Métricas complementarias del Capítulo 5.")
    ap.add_argument('--datos', nargs='+', required=True, help="carpetas de corridas (una por repetición)")
    ap.add_argument('--salida', required=True, help="carpeta de salida")
    args = ap.parse_args(argv)
    filas = []
    for carpeta in args.datos:
        for alg in ALGORITMOS:
            for ruta in RUTAS:
                archivo = os.path.join(carpeta, f"telemetria_{alg}_{ruta}.csv")
                if not os.path.exists(archivo):
                    print(f"  falta {archivo}")
                    continue
                filas.append(dict(algoritmo=alg, ruta=ruta, carpeta=os.path.basename(os.path.normpath(carpeta)),
                                  **metricas(pd.read_csv(archivo), ruta)))
    t = pd.DataFrame(filas)
    os.makedirs(args.salida, exist_ok=True)
    t.to_csv(os.path.join(args.salida, 'metricas_complementarias.csv'), index=False)
    cols = ['sobrepaso_captura_m', 'saturacion_pct', 'amplitud_relativa', 'velocidad_media_ms']
    g = t.groupby(['ruta', 'algoritmo'], sort=False)[cols].agg(['mean', 'std'])
    g.columns = [c if s == 'mean' else f"{c}_sd" for c, s in g.columns]
    g.reset_index().to_csv(os.path.join(args.salida, 'metricas_complementarias_agregadas.csv'), index=False)
    with pd.option_context('display.width', 200, 'display.max_columns', 20):
        print(g.round(3).to_string())
    return 0


if __name__ == "__main__":
    sys.exit(main())
