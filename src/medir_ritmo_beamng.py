"""
medir_ritmo_beamng.py - Mide cuánto tarda cada petición a BeamNG en modo síncrono (no es un experimento).

En modo síncrono la simulación avanza a 0.22 veces el tiempo real (corridas del 04-nov-2026) porque BeamNG
atiende cada petición de Python en su ciclo de cuadro y el lazo hace varias en serie por paso. Este script
abre el mismo escenario que los experimentos, con el camión detenido y el freno de mano puesto, y para
60, 120 y 240 pasos por segundo cronometra:
  - step(1), step(4), step(8): la pendiente contra n es la duración real de un cuadro;
  - la lectura del camión, la del tráiler, las dos en serie y la lectura conjunta (truck_trailer.LECTURA_CONJUNTA);
  - la imagen de una cámara con poll() (petición) y con stream() (memoria compartida);
  - truck.control() (espera la confirmación).
Además comprueba que la lectura conjunta entrega exactamente los mismos valores que las dos lecturas por
separado, y proyecta el ritmo de cada configuración. No escribe archivos.

Con --factor mide, con el Timer (de a 60 cuadros), cuánto tiempo simulado avanza cada cuadro en otras
configuraciones, y el ritmo con la telemetría registrada cada n cuadros (truck_trailer.DECIMACION_TELEMETRIA).
steps_per_second es en realidad el límite de cuadros gráficos de BeamNG (mensaje FPSLimit) y en modo
determinista cada cuadro avanza 1/límite s: por eso un cuadro dura 16.7 ms de reloj con 60. Medido el
04-nov-2026 (RTX 3050 Ti):
  - sin limitador (remove_step_limit): 216 cuadros por segundo, pero cada cuadro avanza el tiempo real que
    tardó (4.3 ± 0.8 ms): deja de ser determinista. Descartado.
  - speedFactor = k (con --speedfactor): cada cuadro avanza 50·k ms en vez de 16.7 y el ritmo empeora
    (0.33x -> 0.19x con k = 4). Descartado.
  - telemetría cada 1 / 2 / 4 / 8 cuadros (misma física): 0.32x / 0.47x / 0.62x / 0.73x.

Uso, desde la raíz del repositorio y con BeamNG cerrado:
    python src/medir_ritmo_beamng.py              (todo, ~3 min)
    python src/medir_ritmo_beamng.py --comprobar  (solo la comparación de lecturas, ~1 min)
    python src/medir_ritmo_beamng.py --factor     (configuraciones de cuadros y telemetría cada n cuadros, ~2 min;
                                                   --speedfactor agrega speedFactor 1..4)
"""
import sys
import time
import numpy as np
from arx_truck_ai import truck_trailer as tt
from arx_truck_ai import bng_open

PASOS_POR_SEGUNDO = (60, 120, 240)
FACTORES_VELOCIDAD = (1, 2, 3, 4)
REPETICIONES = 100
DECIMACION_CONTROL = 8          # muestras de telemetría (1/60 s) por actualización del control


def _cronometrar(funcion, n=REPETICIONES):
    """Tiempo medio [ms] de una llamada (más la desviación estándar)."""
    t = np.empty(n)
    for i in range(n):
        t0 = time.perf_counter()
        funcion()
        t[i] = time.perf_counter() - t0
    return 1e3 * t.mean(), 1e3 * t.std()


def _copia_sensores(tr):
    """Valores actuales de todos los sensores de los dos vehículos (para comparar lecturas)."""
    return {(veh.vid, nombre): repr(sensor.data)
            for veh in (tr.truck, tr.trailer) for nombre, sensor in veh.sensors.items()}


def _lectura_serie(tr):
    tr.truck.sensors.poll()
    tr.trailer.sensors.poll()


def _lectura_conjunta(tr):
    tt.LECTURA_CONJUNTA = True
    tr.leer_vehiculos()


def _comprobar_lectura_conjunta(tr):
    """Tras un step, lee cuatro veces sin avanzar: serie, serie, conjunta, serie. Con la simulación en pausa
    las cuatro deberían coincidir; si solo difiere la primera, es el sensor (el Timer se actualiza un cuadro
    después del step), no la lectura conjunta. Imprime lo que difiera."""
    tr.bng.control.step(1)
    lecturas = []
    for nombre, leer in (('serie 1', _lectura_serie), ('serie 2', _lectura_serie),
                         ('conjunta', _lectura_conjunta), ('serie 3', _lectura_serie)):
        leer(tr)
        lecturas.append((nombre, _copia_sensores(tr), tr.truck.sensors['timer'].get('time')))
    if not tr._lectura_conjunta_ok:
        return 'no disponible en esta versión de BeamNGpy'
    print("\nTimer tras un step, en cuatro lecturas sin avanzar: " +
          ", ".join(f"{n} = {t}" for n, _, t in lecturas))
    ref = lecturas[1][1]
    for nombre, copia, _ in lecturas:
        distintos = [k for k in ref if ref[k] != copia.get(k)]
        print(f"  {nombre:8s} frente a serie 2: {'idéntica' if not distintos else f'difiere en {distintos}'}")
    conj, s3 = lecturas[2][1], lecturas[3][1]
    return 'idénticos' if conj == ref == s3 else 'DISTINTOS (ver arriba)'


def _tiempo_timer(tr):
    """Timer de BeamNG [s]. Se lee dos veces: el Timer se actualiza un cuadro después del step."""
    tr.leer_vehiculos()
    tr.leer_vehiculos()
    return float(tr.truck.sensors['timer']['time'])


def _probar_factor(tr):
    """Para cada configuración: tiempo simulado por cuadro (Timer, 5 tandas de 60 cuadros), duración de un
    cuadro de reloj y ritmo proyectado del lazo (step(1) + lectura conjunta por muestra de 1/60 s)."""
    bng = tr.bng
    tt.LECTURA_CONJUNTA = True
    print("\n=== Configuraciones de cuadros (para que los datos no cambien, cada cuadro debe avanzar 16.667 ms"
          " simulados, igual que la base) ===")
    configuraciones = [('base (actual)', lambda: bng.settings.set_deterministic(60)),
                       ('sin limitador', lambda: (bng.settings.set_deterministic(60), bng.settings.remove_step_limit()))]
    if '--speedfactor' in sys.argv:
        configuraciones += [(f'speedFactor {k}', lambda k=k: bng.settings.set_deterministic(60 * k, speed_factor=k))
                            for k in FACTORES_VELOCIDAD]
    for nombre, configurar in configuraciones:
        configurar()
        bng.control.step(10)                                    # que el límite de cuadros ya rija
        dts = []
        for _ in range(5):
            t0 = _tiempo_timer(tr)
            bng.control.step(60)
            dts.append((_tiempo_timer(tr) - t0) / 60)
        s1 = _cronometrar(lambda: bng.control.step(1), 40)[0]
        s8 = _cronometrar(lambda: bng.control.step(8), 40)[0]
        cuadro = (s8 - s1) / 7
        muestra = s1 + _cronometrar(tr.leer_vehiculos, 40)[0]
        print(f"  {nombre:14s} cada cuadro avanza {1e3 * np.mean(dts):7.3f} ms simulados "
              f"(desv. {1e3 * np.std(dts):.3f}) | cuadro de reloj {cuadro:5.1f} ms ({1e3 / cuadro:4.0f} por s) | "
              f"step(1) + lectura {muestra:5.1f} ms -> {1e3 / 60 / muestra:.2f}x tiempo real")

    # Telemetría a menos de 60 Hz con la misma física: un step(n) y una lectura cada n cuadros, más el control
    # (una petición) cada 8 cuadros. Los comandos cambian en los mismos cuadros que hoy.
    bng.settings.set_deterministic(60)
    bng.control.step(10)
    lectura = _cronometrar(tr.leer_vehiculos, 40)[0]
    control = _cronometrar(lambda: tr.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=1), 40)[0]
    print("\n=== Base con la telemetría registrada cada n cuadros (física idéntica) ===")
    for n in (1, 2, 4, 8):
        paso = _cronometrar(lambda n=n: bng.control.step(n), 40)[0]
        ciclo = (8 // n) * (paso + lectura) + control             # un ciclo de control = 8 cuadros = 0.133 s
        print(f"  cada {n} cuadro(s) -> telemetría a {60 // n:2d} Hz: {ciclo:6.1f} ms por ciclo de control "
              f"-> {1e3 * 8 / 60 / ciclo:.2f}x tiempo real")
    bng.settings.set_deterministic(60)


def main():
    tr, _ = bng_open.main('fwd')
    bng = tr.bng
    tr.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=1)
    bng.control.pause()
    cam = tr.front_cam

    if '--factor' in sys.argv:
        _probar_factor(tr)
        tr.liberar_simulacion()
        input('\nPresione enter para cerrar BeamNG...')
        bng.close()
        return

    resultados = {}
    for sps in (() if '--comprobar' in sys.argv else PASOS_POR_SEGUNDO):
        bng.settings.set_deterministic(sps)
        bng.control.step(10)                                    # que el límite de cuadros ya rija
        print(f"\n=== {sps} pasos por segundo ===")
        r = {}
        for n in (1, 4, 8):
            r[f'step{n}'] = _cronometrar(lambda n=n: bng.control.step(n))
        r['camion'] = _cronometrar(tr.truck.sensors.poll)
        r['trailer'] = _cronometrar(tr.trailer.sensors.poll)
        tt.LECTURA_CONJUNTA = False
        r['serie'] = _cronometrar(tr.leer_vehiculos)
        tt.LECTURA_CONJUNTA = True
        r['conjunta'] = _cronometrar(tr.leer_vehiculos)
        if cam is not None:
            r['cam_poll'] = _cronometrar(cam.poll, 30)
            try:
                r['cam_stream'] = _cronometrar(cam.stream, 30)
            except Exception as e:
                print(f"  Camera.stream no disponible ({type(e).__name__})")
        r['control'] = _cronometrar(lambda: tr.truck.control(steering=0.0, throttle=0.0, brake=1.0, parkingbrake=1))

        pend, ordenada = np.polyfit([1, 4, 8], [r[f'step{n}'][0] for n in (1, 4, 8)], 1)
        r['cuadro'], r['fijo'] = pend, ordenada
        for clave, valor in r.items():
            if not isinstance(valor, tuple):
                continue
            media, desv = valor
            print(f"  {clave:10s} {media:7.1f} ± {desv:5.1f} ms  ({media / pend:4.1f} cuadros)")
        print(f"  duración de un cuadro: {pend:.1f} ms ({1e3 / pend:.0f} cuadros por segundo; "
              f"límite pedido {sps}) | costo fijo de step: {ordenada:.1f} ms")
        resultados[sps] = r

    bng.settings.set_deterministic(60)
    print(f"\nLectura conjunta contra lecturas por separado: {_comprobar_lectura_conjunta(tr)}")

    # Ritmo proyectado por muestra de 1/60 s: step de k cuadros + lectura + (cámara + control) / decimación
    print("\nRitmo proyectado (veces el tiempo real, con cámara y control cada "
          f"{DECIMACION_CONTROL} muestras):")
    print(f"  {'pasos/s':>8s} {'actual':>8s} {'opción A':>9s}")
    for sps, r in resultados.items():
        k = sps // 60
        paso = r['cuadro'] * k + r['fijo']
        extra = (r.get('cam_poll', (0,))[0] + r['control'][0]) / DECIMACION_CONTROL
        extra_a = (r.get('cam_stream', r.get('cam_poll', (0,)))[0] + r['control'][0]) / DECIMACION_CONTROL
        actual = paso + r['serie'][0] + 2 * extra          # hoy se piden las dos cámaras
        opcion_a = paso + r['conjunta'][0] + extra_a
        print(f"  {sps:8d} {1e3 / 60 / actual:7.2f}x {1e3 / 60 / opcion_a:8.2f}x")
    print("  (sin contar la proyección de la ruta ni la ventana de OpenCV, ~1-3 ms por ciclo de control)")

    bng.settings.set_deterministic(60)
    tr.liberar_simulacion()
    input('\nPresione enter para cerrar BeamNG...')
    bng.close()


if __name__ == "__main__":
    main()
