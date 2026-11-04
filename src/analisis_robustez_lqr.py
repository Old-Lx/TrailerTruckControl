"""
analisis_robustez_lqr.py - Estabilidad del LQR implementado sobre la planta identificada (Capítulo 3).

No es un experimento ni usa BeamNG: es un análisis de diseño sobre el modelo lineal, para comprobar que el diseño
"pesimista" del LQR (L1_NOMINAL, L2_NOMINAL, TAU_VOLANTE_NOMINAL en backwards_lqr.py) estabiliza la planta
identificada y cuánta incertidumbre tolera.

Lazo simulado tal como corre en BeamNG (modo síncrono, recta, sin prealimentación):
  - planta lineal de reversa a 60 Hz:  x1' = -v x2 + v (DP/L2) x3,  x2' = (v/L2) x3,  x3' = (v/L2) x3 + v k d
    con k = PHI/L1 identificado y d el ángulo real de las ruedas: comando -> retardo puro -> primer orden tau;
  - control cada 8 pasos (7.5 Hz) con retención de orden cero: integra x4 += x1 T, estima x5 con el modelo de
    primer orden del controlador (TAU_VOLANTE_NOMINAL, sin retardo) y aplica s = -K x_hat;
  - K de la tabla del controlador en |δF2| = 0, programada en V o fija en V0 = 2 m/s.
El sistema muestreado es lineal y periódico: se arma el mapa de un período de control (todas las variables:
planta, actuador, cola del retardo, x4, x5 estimado y comando retenido) y su radio espectral rho decide la
estabilidad. Se reporta la parte real equivalente ln(rho)/T (negativa = estable).

Uso (desde la raíz del repositorio):
    python src/analisis_robustez_lqr.py                                  (parámetros identificados del código)
    python src/analisis_robustez_lqr.py --json corridas/identificacion/out/parametros_identificados.json
    python src/analisis_robustez_lqr.py --barrido 2000                   (+ barrido aleatorio de incertidumbre)
"""
import argparse
import json
import math
import sys

import numpy as np
from scipy.linalg import expm

from arx_truck_ai import backwards_lqr as lqr

DT = 1.0 / 60.0
DECIMACION = 8
T_CONTROL = DECIMACION * DT
VELOCIDADES = (0.5, 1.0, 1.5, 2.0, 2.5)


def planta_desde_json(ruta):
    """Parámetros de la planta real desde parametros_identificados.json (identificar_planta.py)."""
    with open(ruta, encoding='utf-8') as f:
        p = json.load(f)
    tr, rm, pm = p['tractor'], p['remolque'], p['punto_medido']
    return dict(k=abs(tr['k_phi_sobre_L1']), L2=rm['L2_m'], DP=pm['DP_m'],
                retardo=tr['retardo_puro_s'], tau=tr['tau_s'])


def planta_del_codigo():
    """Parámetros identificados que usa hoy el código (backwards_lqr.py)."""
    return dict(k=lqr.PHI_MAX_RAD / lqr.L1_IDENT, L2=lqr.L2_IDENT, DP=lqr.DP_IDENT,
                retardo=lqr.RETARDO_PURO_IDENT, tau=lqr.TAU_IDENT)


def radio_espectral(K, v, planta):
    """Radio espectral del mapa de un período de control del lazo muestreado (ver docstring)."""
    k, L2, DP, tau = planta['k'], planta['L2'], planta['DP'], max(planta['tau'], 1e-3)
    nd = int(round(planta['retardo'] / DT))                 # retardo puro en pasos de 60 Hz
    # Planta continua [x1, x2, x3, d] con entrada u_d (comando retrasado); discretización exacta a 60 Hz
    Ac = np.array([[0.0, -v, v * DP / L2, 0.0],
                   [0.0, 0.0, v / L2, 0.0],
                   [0.0, 0.0, v / L2, v * k],
                   [0.0, 0.0, 0.0, -1.0 / tau]])
    Bc = np.array([0.0, 0.0, 0.0, 1.0 / tau])
    M = np.zeros((5, 5)); M[:4, :4] = Ac; M[:4, 4] = Bc
    E = expm(M * DT)
    Ad, Bd = E[:4, :4], E[:4, 4]
    alfa = 1.0 - math.exp(-T_CONTROL / lqr.TAU_VOLANTE_NOMINAL)
    K = np.asarray(K, dtype=float).ravel()

    # Estado completo: [x1, x2, x3, d, cola (nd), x4, x5_est, u_retenido]
    n = 4 + nd + 3
    i4, i5, iu = 4 + nd, 5 + nd, 6 + nd

    def periodo(z):
        z = z.copy()
        x = z[:4]
        cola = list(z[4:4 + nd])
        x4, x5, u_prev = z[i4], z[i5], z[iu]
        # Ciclo de control (mismo orden que control_lqr_reversa): integrador, estimación de x5, ley
        x4 = x4 + x[0] * T_CONTROL
        x5 = x5 + alfa * (u_prev - x5)
        u = -float(K @ np.array([x[0], x[1], x[2], x4, x5]))
        for _ in range(DECIMACION):                         # 8 pasos de 60 Hz con u retenido
            cola.append(u)
            ud = cola.pop(0) if nd else u
            x = Ad @ x + Bd * ud
        out = np.zeros(n)
        out[:4] = x
        out[4:4 + nd] = cola
        out[i4], out[i5], out[iu] = x4, x5, u
        return out

    Phi = np.column_stack([periodo(e) for e in np.eye(n)])
    return float(np.max(np.abs(np.linalg.eigvals(Phi))))


def parte_real(rho):
    return math.log(max(rho, 1e-12)) / T_CONTROL


def tabla(planta, titulo):
    tab_v = lqr.precalcular_ganancias_lqr(2.0, lqr.L2_NOMINAL, velocidades=lqr.PUNTOS_OPERACION_V)
    tab_fija = lqr.precalcular_ganancias_lqr(2.0, lqr.L2_NOMINAL)
    print(f"\n{titulo}: PHI/L1 = {planta['k']:.4f} 1/m (L1 = {lqr.PHI_MAX_RAD / planta['k']:.2f} m con PHI = "
          f"{lqr.PHI_MAX_RAD}), L2 = {planta['L2']:.2f} m, DP = {planta['DP']:.2f} m, retardo {planta['retardo']:.2f} s "
          f"+ tau {planta['tau']:.2f} s")
    print(f"  {'v [m/s]':>8s} {'K programada en V':>22s} {'K fija en 2 m/s':>20s}")
    for v in VELOCIDADES:
        r_v = radio_espectral(lqr.interpolar_ganancia_lqr(0.0, tab_v, v), v, planta)
        r_f = radio_espectral(lqr.interpolar_ganancia_lqr(0.0, tab_fija), v, planta)
        print(f"  {v:8.1f} {parte_real(r_v):+12.3f} 1/s {'(estable)' if r_v < 1 else '(INESTABLE)':>10s}"
              f" {parte_real(r_f):+10.3f} 1/s {'(estable)' if r_f < 1 else '(INESTABLE)':>10s}")


def barrido(planta, n, semilla=0):
    """Fracción de plantas estables con incertidumbre uniforme alrededor de la identificada:
    PHI/L1 ±20 %, L2 ±20 %, DP ±10 %, retardo y tau ±30 %, en todas las velocidades."""
    rng = np.random.default_rng(semilla)
    tab_v = lqr.precalcular_ganancias_lqr(2.0, lqr.L2_NOMINAL, velocidades=lqr.PUNTOS_OPERACION_V)
    Ks = {v: lqr.interpolar_ganancia_lqr(0.0, tab_v, v) for v in VELOCIDADES}
    fallas = {v: 0 for v in VELOCIDADES}
    for _ in range(n):
        p = dict(k=planta['k'] * rng.uniform(0.8, 1.2), L2=planta['L2'] * rng.uniform(0.8, 1.2),
                 DP=planta['DP'] * rng.uniform(0.9, 1.1), retardo=planta['retardo'] * rng.uniform(0.7, 1.3),
                 tau=planta['tau'] * rng.uniform(0.7, 1.3))
        for v in VELOCIDADES:
            if radio_espectral(Ks[v], v, p) >= 1.0:
                fallas[v] += 1
    print(f"\nBarrido de incertidumbre ({n} plantas; PHI/L1 y L2 ±20 %, DP ±10 %, retardo y tau ±30 %), K programada en V:")
    for v in VELOCIDADES:
        print(f"  v = {v:.1f} m/s: estables {100 * (1 - fallas[v] / n):5.1f} %")


def main(argv=None):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    ap = argparse.ArgumentParser(description="Estabilidad del LQR implementado sobre la planta identificada.")
    ap.add_argument('--json', help="parametros_identificados.json de identificar_planta.py")
    ap.add_argument('--barrido', type=int, default=0, help="número de plantas del barrido de incertidumbre")
    args = ap.parse_args(argv)
    print(f"Diseño del LQR (pesimista): L1 = {lqr.L1_NOMINAL} m, L2 = {lqr.L2_NOMINAL} m, DP = {lqr.DP_NOMINAL} m, "
          f"tau = {lqr.TAU_VOLANTE_NOMINAL} s; control a {1 / T_CONTROL:.1f} Hz")
    planta = planta_desde_json(args.json) if args.json else planta_del_codigo()
    tabla(planta, "Planta identificada" + (f" ({args.json})" if args.json else " (valores actuales del código)"))
    if args.barrido:
        barrido(planta, args.barrido)
    return 0


if __name__ == "__main__":
    sys.exit(main())
