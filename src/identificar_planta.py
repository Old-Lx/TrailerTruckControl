"""
identificar_planta.py - Identificación de los parámetros de la planta a partir de identificacion_planta.csv.

Modelo (cinemático, sin deslizamiento; theta = psi_tractor - psi_remolque, v = velocidad del tractor):
  Volante de la cabina    s_real(t) = G * [retardo puro d] -> [primer orden tau] de s_cmd(t)
                          (s_real: ángulo del volante del sensor Electrics, en grados; G en grados por unidad).
                          En BeamNG es -900°/unidad con UN cuadro de retraso (corrida del 30-oct-2026): copia el
                          comando y NO incluye la dinámica de las ruedas ni de los neumáticos.
  Tractor                 psi1_dot = v * k * [retardo puro d] -> [primer orden tau] de s_cmd(t),  k = PHI/L1
                          d y tau se buscan en rejilla y k por mínimos cuadrados: es el retardo EFECTIVO entre
                          el comando y el giro del tractor, el que modela x5 en el LQR. (También se reporta el
                          ajuste con el volante de la cabina como entrada, para comparar.)
  Remolque (enganche      psi2_dot = (v*sin(theta) - M*psi1_dot*cos(theta)) / L2
  desplazado M detrás del eje trasero del tractor; M = 0 es la hipótesis "on-axle" del Capítulo 1)
  Punto medido            v_lat = DP * psi2_dot   (velocidad lateral del punto de control del remolque,
                          que está a DP por delante del eje del remolque)
Cada relación es lineal en sus parámetros: se ajustan por mínimos cuadrados (el actuador, por búsqueda en
rejilla de d y tau con G por mínimos cuadrados). Las derivadas se calculan con un filtro Savitzky-Golay.

Uso:
    python src/identificar_planta.py identificacion_planta.csv --salida figuras/cap3
    python src/identificar_planta.py identificacion_planta.csv --validar telemetria_lqr_recta_y.csv
Salidas: parametros_identificados.json y figuras identificacion_*.png/pdf en --salida.
--validar comprueba el modelo del remolque y del punto medido con una corrida en REVERSA (telemetría normal).
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import lfilter, lfilter_zi, savgol_filter

PHI_MAX_SUPUESTO = 0.6       # rad: el código usa este ángulo de rueda con |steering| = 1 (para expresar L1)
VENTANA_DERIVADA_S = 0.5


def derivada(x, dt):
    n = max(5, int(round(VENTANA_DERIVADA_S / dt)) | 1)
    return savgol_filter(x, n, 2, deriv=1, delta=dt)


def r2(y, y_hat):
    y, y_hat = np.asarray(y), np.asarray(y_hat)
    ss = np.sum((y - np.mean(y)) ** 2)
    return float(1.0 - np.sum((y - y_hat) ** 2) / ss) if ss > 0 else float('nan')


def primer_orden(u, dt, tau, retardo):
    """Respuesta de [retardo puro] + [primer orden de ganancia 1] a la señal u (muestreada a dt)."""
    n = int(round(retardo / dt))
    u_r = np.concatenate((np.full(n, u[0]), u[:len(u) - n])) if n > 0 else u
    a = 1.0 - math.exp(-dt / tau) if tau > 1e-6 else 1.0
    # y[i] = (1-a)*y[i-1] + a*u[i], arrancando en régimen con u[0]
    y, _ = lfilter([a], [1.0, -(1.0 - a)], u_r, zi=lfilter_zi([a], [1.0, -(1.0 - a)]) * u_r[0])
    return y


def identificar_actuador(t, cmd, real, dt):
    mejor = None
    for retardo in np.arange(0.0, 1.0 + 1e-9, dt):
        for tau in np.arange(0.02, 1.5, 0.02):
            y = primer_orden(cmd, dt, tau, retardo)
            G = float(np.dot(y, real) / np.dot(y, y)) if np.dot(y, y) > 0 else 0.0
            e = float(np.mean((real - G * y) ** 2))
            if mejor is None or e < mejor[0]:
                mejor = (e, retardo, tau, G)
    _, retardo, tau, G = mejor
    return dict(retardo_puro_s=float(retardo), tau_s=float(tau), ganancia_grados=G,
                retardo_equivalente_s=float(retardo + tau),
                r2=r2(real, G * primer_orden(cmd, dt, tau, retardo)))


def identificar_giro(cmd, v, psi1_d, mov, dt):
    """Retardo efectivo del volante visto en el giro del tractor: psi1_dot = v*k*[retardo + primer orden](cmd)."""
    y = psi1_d[mov]
    mejor = None
    for retardo in np.arange(0.0, 1.0 + 1e-9, dt):
        for tau in np.arange(0.02, 1.5, 0.02):
            X = (v * primer_orden(cmd, dt, tau, retardo))[mov]
            xx = float(np.dot(X, X))
            if xx <= 0:
                continue
            k = float(np.dot(X, y) / xx)
            e = float(np.mean((y - k * X) ** 2))
            if mejor is None or e < mejor[0]:
                mejor = (e, retardo, tau, k)
    _, retardo, tau, k = mejor
    return k, float(retardo), float(tau)


def cargar(archivo):
    df = pd.read_csv(archivo)
    df = df.dropna(subset=['t']).sort_values('t').reset_index(drop=True)
    return df


def identificar(df):
    t = df['t'].to_numpy(float)
    dt = float(np.median(np.diff(t)))
    v = df['velocidad'].to_numpy(float)
    psi1 = np.unwrap(df['psi_tractor'].to_numpy(float))
    psi2 = np.unwrap(df['psi_trailer'].to_numpy(float))
    theta = np.arctan2(np.sin(psi1 - psi2), np.cos(psi1 - psi2))
    psi1_d, psi2_d = derivada(psi1, dt), derivada(psi2, dt)
    cmd = df['steering_cmd'].to_numpy(float)
    mov = v > 0.5 * np.nanmax(v)                     # solo con el vehículo en marcha
    res = dict(muestras=int(len(df)), dt_s=dt)
    figs = {}

    # 1) Volante de la cabina (sensor Electrics)
    real = df['volante_real_deg'].to_numpy(float) if 'volante_real_deg' in df else np.full(len(df), np.nan)
    s_act = None
    if np.isfinite(real).mean() > 0.9:
        act = identificar_actuador(t, cmd, np.nan_to_num(real), dt)
        act['nota'] = ("ángulo del volante de la cabina; si el retardo es ~1 cuadro, el sensor solo copia el "
                       "comando y la dinámica real del volante está en el ajuste del tractor")
        s_act = np.nan_to_num(real) / act['ganancia_grados']
        res['actuador'] = act
        figs['actuador'] = (t, cmd, real / act['ganancia_grados'],
                            primer_orden(cmd, dt, act['tau_s'], act['retardo_puro_s']))
    else:
        res['actuador'] = None              # sin sensor Electrics

    # 2) Tractor: psi1_dot = v*k*[retardo + primer orden](cmd)  (retardo efectivo comando -> giro)
    k, retardo, tau = identificar_giro(cmd, v, psi1_d, mov, dt)
    X = (v * primer_orden(cmd, dt, tau, retardo))[mov]
    res['tractor'] = dict(k_phi_sobre_L1=k, L1_equivalente_m=PHI_MAX_SUPUESTO / abs(k) if k else float('nan'),
                          retardo_puro_s=retardo, tau_s=tau, retardo_equivalente_s=retardo + tau,
                          nota=f"L1 equivalente suponiendo PHI_MAX = {PHI_MAX_SUPUESTO} rad (solo PHI/L1 es "
                               "identificable); retardo y tau: del comando al giro del tractor (x5 del LQR)",
                          r2=r2(psi1_d[mov], k * X))
    if s_act is not None:                   # comparación: el volante de la cabina como entrada, sin retardo extra
        Xc = (v * s_act)[mov]
        kc = float(np.dot(Xc, psi1_d[mov]) / np.dot(Xc, Xc))
        res['tractor'].update(k_con_volante_cabina=kc, r2_con_volante_cabina=r2(psi1_d[mov], kc * Xc))
    figs['tractor'] = (t[mov], psi1_d[mov], k * X)

    # 3) Remolque: psi2_dot = a*(v sin theta) + b*(psi1_dot cos theta);  L2 = 1/a, M = -b/a
    A = np.column_stack(((v * np.sin(theta))[mov], (psi1_d * np.cos(theta))[mov]))
    (a, b), *_ = np.linalg.lstsq(A, psi2_d[mov], rcond=None)
    a0 = float(np.dot(A[:, 0], psi2_d[mov]) / np.dot(A[:, 0], A[:, 0]))
    res['remolque'] = dict(L2_m=1.0 / a, M_enganche_m=-b / a, r2=r2(psi2_d[mov], A @ np.array([a, b])),
                           L2_on_axle_m=1.0 / a0, r2_on_axle=r2(psi2_d[mov], a0 * A[:, 0]))
    figs['remolque'] = (t[mov], psi2_d[mov], A @ np.array([a, b]))

    # 4) Punto medido del remolque: v_lat = DP*psi2_dot
    xr, yr = df['x_trailer'].to_numpy(float), df['y_trailer'].to_numpy(float)
    vx, vy = derivada(xr, dt), derivada(yr, dt)
    v_lat = -np.sin(psi2) * vx + np.cos(psi2) * vy          # componente hacia la izquierda del remolque
    DP = float(np.dot(psi2_d[mov], v_lat[mov]) / np.dot(psi2_d[mov], psi2_d[mov]))
    res['punto_medido'] = dict(DP_m=DP, r2=r2(v_lat[mov], DP * psi2_d[mov]))
    figs['punto'] = (psi2_d[mov], v_lat[mov], DP)
    return res, figs


def validar_reversa(archivo, L2, M, DP, k=None):
    """Comprueba el modelo del remolque y del punto medido con una corrida en reversa (telemetría estándar)."""
    df = cargar(archivo)
    t = df['t'].to_numpy(float)
    dt = float(np.median(np.diff(t)))
    v = -df['velocidad'].to_numpy(float)                   # reversa: velocidad con signo negativo
    psi1 = np.unwrap(df['psi_tractor'].to_numpy(float))
    psi2 = np.unwrap(df['psi_trailer'].to_numpy(float))
    theta = np.arctan2(np.sin(psi1 - psi2), np.cos(psi1 - psi2))
    psi1_d, psi2_d = derivada(psi1, dt), derivada(psi2, dt)
    mov = np.abs(v) > 0.3
    pred = (v * np.sin(theta) - M * psi1_d * np.cos(theta)) / L2
    xr, yr = df['x_trailer'].to_numpy(float), df['y_trailer'].to_numpy(float)
    v_lat = -np.sin(psi2) * derivada(xr, dt) + np.cos(psi2) * derivada(yr, dt)
    return dict(archivo=os.path.basename(archivo), r2_remolque=r2(psi2_d[mov], pred[mov]),
                r2_punto_medido=r2(v_lat[mov], DP * psi2_d[mov])), (t[mov], psi2_d[mov], pred[mov])


def figuras(figs, salida, formatos, validacion=None):
    rutas = []

    def guardar(fig, nombre):
        for ext in formatos:
            p = os.path.join(salida, f"{nombre}.{ext}")
            fig.savefig(p, dpi=300, bbox_inches='tight')
            rutas.append(p)
        plt.close(fig)

    if 'actuador' in figs:
        t, cmd, real, mod = figs['actuador']
        fig, ax = plt.subplots(figsize=(7, 3))
        ax.plot(t, cmd, lw=1, label='Comando')
        ax.plot(t, real, lw=1.2, label='Volante real (normalizado)')
        ax.plot(t, mod, '--', lw=1, label='Modelo: retardo + primer orden')
        ax.set_xlabel('t [s]'); ax.set_ylabel('Volante de la cabina [-1, 1]'); ax.legend(); ax.grid(alpha=.3)
        guardar(fig, 'identificacion_actuador')
    for clave, ylabel in (('tractor', r'$\dot\psi_1$ [rad/s]'), ('remolque', r'$\dot\psi_2$ [rad/s]')):
        t, med, mod = figs[clave]
        fig, ax = plt.subplots(figsize=(7, 3))
        ax.plot(t, med, lw=1.2, label='Medido (BeamNG)')
        ax.plot(t, mod, '--', lw=1, label='Modelo identificado')
        ax.set_xlabel('t [s]'); ax.set_ylabel(ylabel); ax.legend(); ax.grid(alpha=.3)
        guardar(fig, f'identificacion_{clave}')
    w, vl, DP = figs['punto']
    fig, ax = plt.subplots(figsize=(4, 4))
    ax.plot(w, vl, '.', ms=2, alpha=.4, label='Medido')
    xs = np.linspace(np.min(w), np.max(w), 2)
    ax.plot(xs, DP * xs, 'k--', label=f'DP = {DP:.2f} m')
    ax.set_xlabel(r'$\dot\psi_2$ [rad/s]'); ax.set_ylabel('Velocidad lateral del punto medido [m/s]')
    ax.legend(); ax.grid(alpha=.3)
    guardar(fig, 'identificacion_punto_medido')
    if validacion is not None:
        t, med, mod = validacion
        fig, ax = plt.subplots(figsize=(7, 3))
        ax.plot(t, med, lw=1.2, label='Medido en reversa (BeamNG)')
        ax.plot(t, mod, '--', lw=1, label='Modelo identificado en marcha adelante')
        ax.set_xlabel('t [s]'); ax.set_ylabel(r'$\dot\psi_2$ [rad/s]'); ax.legend(); ax.grid(alpha=.3)
        guardar(fig, 'identificacion_validacion_reversa')
    return rutas


def main(argv=None):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    ap = argparse.ArgumentParser(description="Identificación de la planta (Capítulo 3).")
    ap.add_argument('archivo', help="identificacion_planta.csv")
    ap.add_argument('--salida', default='.')
    ap.add_argument('--formatos', nargs='+', default=['png', 'pdf'])
    ap.add_argument('--validar', help="telemetría de una corrida en reversa para validar el modelo del remolque")
    args = ap.parse_args(argv)
    os.makedirs(args.salida, exist_ok=True)

    res, figs = identificar(cargar(args.archivo))
    val = None
    if args.validar:
        res['validacion_reversa'], val = validar_reversa(args.validar, res['remolque']['L2_m'],
                                                          res['remolque']['M_enganche_m'], res['punto_medido']['DP_m'])
    res['archivo'] = os.path.basename(args.archivo)
    with open(os.path.join(args.salida, 'parametros_identificados.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    rutas = figuras(figs, args.salida, args.formatos, val)

    act, tr, rm, pm = res['actuador'], res['tractor'], res['remolque'], res['punto_medido']
    print("PARÁMETROS IDENTIFICADOS")
    if act:
        print(f"  volante de la cabina (Electrics): retardo puro {act['retardo_puro_s']:.3f} s + primer orden tau "
              f"{act['tau_s']:.2f} s, G = {act['ganancia_grados']:.1f}°/unidad, R² = {act['r2']:.3f}")
    else:
        print("  volante de la cabina: sin datos del sensor Electrics")
    print(f"  tractor  : PHI/L1 = {tr['k_phi_sobre_L1']:.4f} 1/m  (L1 = {tr['L1_equivalente_m']:.2f} m con PHI = "
          f"{PHI_MAX_SUPUESTO} rad), R² = {tr['r2']:.3f}")
    print(f"             retardo efectivo comando -> giro: puro {tr['retardo_puro_s']:.2f} s + primer orden tau "
          f"{tr['tau_s']:.2f} s (equivalente {tr['retardo_equivalente_s']:.2f} s)")
    if 'r2_con_volante_cabina' in tr:
        print(f"             con el volante de la cabina como entrada: PHI/L1 = {tr['k_con_volante_cabina']:.4f}, "
              f"R² = {tr['r2_con_volante_cabina']:.3f}")
    print(f"  remolque : L2 = {rm['L2_m']:.2f} m, desplazamiento del enganche M = {rm['M_enganche_m']:+.2f} m, "
          f"R² = {rm['r2']:.3f}  (on-axle: L2 = {rm['L2_on_axle_m']:.2f} m, R² = {rm['r2_on_axle']:.3f})")
    print(f"  punto    : DP = {pm['DP_m']:.2f} m, R² = {pm['r2']:.3f}")
    if 'validacion_reversa' in res:
        vr = res['validacion_reversa']
        print(f"  validación en reversa ({vr['archivo']}): R² remolque = {vr['r2_remolque']:.3f}, "
              f"R² punto medido = {vr['r2_punto_medido']:.3f}")
    print(f"Escrito: {os.path.join(args.salida, 'parametros_identificados.json')} y {len(rutas)} figuras")
    return 0


if __name__ == "__main__":
    sys.exit(main())
