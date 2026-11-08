# Campaña del Capítulo 5 (código `v1.0-tesis`)

3 algoritmos (forward con Pure Pursuit, backward aditivo, backward LQR) × 7 rutas × 3 repeticiones = 63 corridas en
BeamNG.tech 0.38.3, todas completadas (sin jackknife ni tiempo máximo).

## Configuración

- Modo síncrono: física a 2000 Hz, 60 pasos por segundo, control a 7.5 Hz con retención de orden cero.
- Telemetría a 7.5 Hz (`DECIMACION_TELEMETRIA = 8`, una muestra por ciclo de control).
- Espera de reposo con freno de mano antes de la primera muestra (`ESPERAR_REPOSO`).
- Velocidad objetivo de 2 m/s con PI y anti-windup de arranque.
- LQR con integrador `corregido`, prealimentación de curvatura y programación de ganancias en V. Los demás valores son
  los de `main.py` en el tag.

## Origen de las corridas

| Carpeta | Corridas | Código |
|---|---|---|
| `campana_r1` | 11 del 07-nov y 10 del 04-nov | `v1.0-tesis` |
| `campana_r2` | 21 del 07-nov | `v1.0-tesis-1-g1efe5b1` |
| `campana_r3` | 21 del 07-nov | `v1.0-tesis-1-g1efe5b1` |

**Corridas del 04-nov reutilizadas en `campana_r1`.** Son las corridas por defecto de los A/B, hechas con controladores
idénticos a los del tag:

- forward: `recta_y`, `parabola_x`, `seno_x`
- backward: `recta_y`, `parabola_x`, `seno_x`
- lqr: `recta_x`, `recta_y`, `parabola_x`, `seno_x`

La fecha de cada corrida está en su JSON. El commit `1efe5b1` solo modifica `matlab/modelaje.m`.

## Archivos

- `telemetria_campana.zip`: telemetría cruda (CSV y JSON), rutas y `version.txt` de las tres carpetas.
- `evaluacion/`: salida de `evaluacion_resultados.py` y de `metricas_complementarias.py`:
  - `metricas.csv`, `metricas_agregadas.csv`, `tabla_metricas.md` y `tabla_metricas.tex`;
  - figuras de trayectorias y respuestas temporales (de `campana_r1`);
  - `metricas_complementarias*.csv`: sobrepaso de captura, saturación del volante, amplitud relativa en las senoides
    y velocidad media.

## Reproducir la evaluación

Desde la raíz del repositorio, después de descomprimir el zip en `corridas/`:

```
python src/evaluacion_resultados.py --datos corridas/campana_r1 corridas/campana_r2 corridas/campana_r3 --salida <carpeta>
python src/metricas_complementarias.py --datos corridas/campana_r1 corridas/campana_r2 corridas/campana_r3 --salida <carpeta>
```
