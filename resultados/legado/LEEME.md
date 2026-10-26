# Datos legados (NO usar en el Capítulo 5)

Corridas de octubre de 2026 hechas con versiones anteriores del código. Se conservan solo como registro.

- Formato anterior de telemetría: 9 columnas, sin `.json` de metadatos ni `ruta_*.csv`.
- Modo libre (tiempo de reloj, ~6-7 Hz), no el modo síncrono a 60 Hz actual.
- `telemetria_lqr_recta.csv` corrió sobre una recta paralela a Y (termina en x = 20, y = 58)
  aunque el nombre dice "recta".
- `telemetria_sistema.log` y `respuesta_sistema.png` corresponden a la corrida de `telemetria_backward_recta.csv`.

Los resultados de la tesis se regeneran en el simulador con el código actual y se guardan en `resultados/`.
