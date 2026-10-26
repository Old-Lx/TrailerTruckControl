# TrailerTruckControl

Código de la tesis *"Diseño de un controlador para maniobras de camiones autónomos con tráiler"*
(Ingeniería Electrónica, Universidad Simón Bolívar). Tutor: Dr. Gerardo Fernández.

Se implementan y comparan tres controladores de seguimiento de ruta para un camión con tráiler,
evaluados en el simulador de física de cuerpos blandos **BeamNG.tech** mediante **BeamNGpy**:

| Algoritmo | Archivo | Punto de control |
|---|---|---|
| Marcha adelante, persecución con *lookahead* dinámico | `src/arx_truck_ai/forward.py` | tractor |
| Marcha atrás, control aditivo con histéresis anti-*jackknife* | `src/arx_truck_ai/backward.py` | remolque |
| Marcha atrás, LQR con programación de ganancias | `src/arx_truck_ai/backwards_lqr.py` | remolque |

## Requisitos

- Windows con **BeamNG.tech v0.38.3** (licencia de investigación).
- Python 3.11 y las dependencias de `requirements.txt` (BeamNGpy 1.35 corresponde a BeamNG.tech 0.38).

```bash
pip install -r requirements.txt
cp .env.example .env      # y editar BNG_HOME / BNG_USER con la instalación local
```

## Ejecutar un experimento

1. En `src/main.py` elegir la ruta (`TIPO_RUTA`, 0-6) y el algoritmo (`ALGORITMO`).
2. Desde la raíz del repositorio:

   ```bash
   python src/main.py
   ```

   BeamNG se abre, se arma el escenario en `smallgrid` y la corrida avanza en modo síncrono
   (pasos de 1/60 s; telemetría a 60 Hz y control a 7.5 Hz). Al terminar se escriben en la carpeta actual
   `telemetria_<algoritmo>_<ruta>.csv`, su `.json` de metadatos y `ruta_<ruta>.csv`.

3. Con todas las corridas hechas, calcular métricas, tablas y figuras:

   ```bash
   python src/evaluacion_resultados.py --datos . --salida figuras/cap5
   ```

Rutas disponibles: `0` recta ∥ X, `1` recta ∥ Y, `2` recta x = y, `3` parábola y = a·x²,
`4` parábola x = a·y², `5` senoide y = A·sin(ωx), `6` senoide x = A·sin(ωy).

### Opciones de los controladores

En `src/main.py`: ley de dirección de marcha adelante (`proporcional` | `pure_pursuit`) y, para el LQR,
prealimentación de curvatura, programación de ganancias en V y modo del integrador. Cada corrida guarda las
opciones usadas en su `.json`. Para comparar variantes, córrelas en carpetas distintas y evalúa cada una con `--datos`.

## Identificación de la planta

```bash
python src/experimento_identificacion.py                 # en BeamNG: maniobras en lazo abierto, marcha adelante
python src/identificar_planta.py identificacion_planta.csv --salida figuras/cap3 \
       --validar telemetria_lqr_recta_y.csv               # opcional: valida el modelo con una corrida en reversa
```

Identifica la dinámica del volante (retardo puro + primer orden), PHI/L1 del tractor, L2 y el desplazamiento
del enganche del remolque, y la distancia DP del punto de control al eje del remolque. Escribe
`parametros_identificados.json` y las figuras de comparación medido/modelo.

## Verificación sin BeamNG (solo para el software)

`src/arx_truck_ai/arnes.py` reemplaza BeamNG por un modelo cinemático y ejecuta los `main()` reales de
punta a punta. Sirve para comprobar que el código no se rompió tras un cambio. **Sus resultados son
sintéticos y no son resultados de la tesis**: los resultados válidos provienen únicamente del simulador.

```bash
mkdir prueba && cd prueba
python ../src/arx_truck_ai/arnes.py <forward|backward|lqr> <tipo_ruta 0..6>
```

## Estructura

```
src/
  main.py                      lanzador de experimentos
  evaluacion_resultados.py     métricas (captura, RMSE lateral y angular, articulación máxima, suavidad), tablas y figuras
  experimento_identificacion.py, identificar_planta.py   identificación de la planta en BeamNG
  arx_truck_ai/
    truck_trailer.py           escenario, sensores, calibración de cámaras, rutas, lectura del estado
    bng_open.py                arranque de BeamNG y del escenario
    forward.py, backward.py, backwards_lqr.py   controladores
    fin_ruta.py                fin de ruta común a los tres algoritmos
    geometria_ruta.py          curvatura de la ruta (prealimentación del LQR)
    debugging_sys.py           telemetría (CSV/JSON) y figuras de apoyo
    arnes.py                   arnés de integración sin BeamNG
matlab/modelaje.m              modelo nominal linealizado (figuras del Capítulo 3)
resultados/                    resultados versionados (legado/: corridas antiguas, no válidas)
docs/beamng/                   listados de vehículos y escenarios de BeamNG
```

## Licencia

GPL-3.0 (ver `LICENSE`).
