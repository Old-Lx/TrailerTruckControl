| Trayectoria | Algoritmo | t_captura [s] | t_asent [s] | RMSE_lat seg. [m] | RMSE_yaw seg. [rad] | θ_max [°] | S_smooth [–] | Error final [m] | Resultado | Duración [s] |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---:|
| Recta ∥ X | Forward (Pure Pursuit) | **0.0** ± 0.0 | **17.8** ± 0.4 | 0.121 ± 0.001 | **0.004** ± 0.000 | **1.35** ± 0.05 | **0.0002** ± 0.0000 | 0.50 ± 0.06 | Completada (3/3) | 111.7 |
|  | Backward (Aditivo) | **0.0** ± 0.0 | 64.7 ± 31.9 | **0.120** ± 0.002 | 0.005 ± 0.000 | 1.54 ± 0.10 | 0.0017 ± 0.0001 | 0.33 ± 0.03 | Completada (3/3) | 111.5 |
|  | Backward (LQR) | **0.0** ± 0.0 | 29.2 ± 0.1 | 0.129 ± 0.001 | 0.012 ± 0.000 | 2.50 ± 0.03 | 0.0014 ± 0.0000 | **0.30** ± 0.01 | Completada (3/3) | 111.6 |
| Recta ∥ Y | Forward (Pure Pursuit) | 23.6 ± 0.2 | **32.6** ± 1.8 | 0.088 ± 0.005 | **0.008** ± 0.001 | 25.80 ± 0.09 | **0.0013** ± 0.0000 | 0.55 ± 0.01 | Completada (3/3) | 109.2 |
|  | Backward (Aditivo) | 51.4 ± 0.3 | 88.6 ± 6.2 | 0.144 ± 0.005 | 0.014 ± 0.001 | **20.92** ± 0.53 | 0.0224 ± 0.0007 | 0.36 ± 0.03 | Completada (3/3) | 114.8 |
|  | Backward (LQR) | **18.4** ± 0.0 | 40.9 ± 0.1 | **0.072** ± 0.001 | 0.010 ± 0.000 | 32.91 ± 0.08 | 0.0034 ± 0.0000 | **0.31** ± 0.01 | Completada (3/3) | 110.7 |
| Recta x = y | Forward (Pure Pursuit) | **15.6** ± 0.0 | 49.5 ± 3.8 | **0.064** ± 0.001 | 0.053 ± 0.000 | **11.64** ± 0.00 | **0.0005** ± 0.0000 | 0.51 ± 0.04 | Completada (3/3) | 152.6 |
|  | Backward (Aditivo) | 27.3 ± 0.0 | 67.0 ± 0.2 | 0.123 ± 0.001 | 0.012 ± 0.000 | 15.04 ± 0.02 | 0.0078 ± 0.0000 | 0.37 ± 0.01 | Completada (3/3) | 153.2 |
|  | Backward (LQR) | 16.8 ± 0.0 | **44.9** ± 5.9 | **0.064** ± 0.001 | **0.009** ± 0.000 | 26.74 ± 0.05 | 0.0038 ± 0.0000 | **0.32** ± 0.01 | Completada (3/3) | 153.6 |
| Parábola y = a·x² | Forward (Pure Pursuit) | 4.2 ± 0.1 | — | **0.261** ± 0.000 | 0.050 ± 0.000 | **5.81** ± 0.03 | **0.0005** ± 0.0000 | 0.54 ± 0.03 | Completada (3/3) | 65.1 |
|  | Backward (Aditivo) | **0.0** ± 0.0 | — | 0.664 ± 0.001 | **0.046** ± 0.000 | 6.64 ± 0.00 | 0.0058 ± 0.0000 | 0.42 ± 0.05 | Completada (3/3) | 65.1 |
|  | Backward (LQR) | **0.0** ± 0.0 | **46.5** ± 3.1 | 0.603 ± 0.005 | 0.072 ± 0.001 | 15.99 ± 0.08 | 0.0264 ± 0.0001 | **0.33** ± 0.02 | Completada (3/3) | 64.9 |
| Parábola x = a·y² | Forward (Pure Pursuit) | 26.0 ± 0.1 | — | 0.174 ± 0.001 | 0.047 ± 0.000 | **21.97** ± 0.12 | **0.0021** ± 0.0000 | 0.50 ± 0.04 | Completada (3/3) | 62.5 |
|  | Backward (Aditivo) | 49.5 ± 0.2 | — | 0.174 ± 0.001 | **0.026** ± 0.000 | 24.48 ± 1.48 | 0.0308 ± 0.0003 | 0.40 ± 0.03 | Completada (3/3) | 65.9 |
|  | Backward (LQR) | **18.3** ± 0.0 | **47.1** ± 2.0 | **0.115** ± 0.006 | 0.041 ± 0.001 | 30.70 ± 0.03 | 0.0254 ± 0.0000 | **0.35** ± 0.01 | Completada (3/3) | 64.7 |
| Senoide y = A·sin(ωx) | Forward (Pure Pursuit) | — | — | 2.223 ± 0.003* | 0.257 ± 0.000* | **9.14** ± 0.02 | **0.0039** ± 0.0000 | 0.80 ± 0.02 | Completada (3/3) | 112.9 |
|  | Backward (Aditivo) | — | — | **1.859** ± 0.003* | **0.205** ± 0.002* | 12.05 ± 0.08 | 0.0516 ± 0.0001 | 1.29 ± 0.04 | Completada (3/3) | 114.3 |
|  | Backward (LQR) | — | — | 2.294 ± 0.004* | 0.329 ± 0.000* | 40.73 ± 0.43 | 0.1619 ± 0.0011 | **0.62** ± 0.02 | Completada (3/3) | 138.0 |
| Senoide x = A·sin(ωy) | Forward (Pure Pursuit) | — | — | 4.226 ± 0.004* | 0.451 ± 0.000* | 27.82 ± 0.07 | **0.0044** ± 0.0000 | 0.79 ± 0.04 | Completada (3/3) | 111.0 |
|  | Backward (Aditivo) | **49.5** ± 0.0 (2/3) | — | 4.641 ± 0.009* | **0.377** ± 0.001* | **26.03** ± 0.08 | 0.0599 ± 0.0002 | 1.21 ± 0.01 | Completada (3/3) | 117.1 |
|  | Backward (LQR) | — | — | **3.452** ± 0.003* | 0.397 ± 0.000* | 40.59 ± 0.59 | 0.1636 ± 0.0009 | **0.66** ± 0.02 | Completada (3/3) | 137.5 |

En negrita, el mejor valor de cada trayectoria entre las corridas completadas.
t_captura: tiempo hasta que el conjunto queda sobre la ruta (|x1| < 0.5 m durante 3 s seguidos). t_asent: desde cuándo |x1| < 0.05 m sin volver a salir (—: nunca). RMSE_lat/RMSE_yaw seg.: solo después de la captura. RMSE_lat: punto de control (tractor en marcha adelante, remolque en reversa). RMSE_yaw: error angular del remolque respecto de la tangente de la ruta. θ_max: toda la corrida. S_smooth = 1/(N−1)·Σ|u_{k+1} − u_k|. Error final: distancia del punto de control al último punto de la ruta, ya detenido. *: el conjunto nunca entró en la banda (en al menos una corrida del caso); métrica sobre toda la corrida. (k/n): la métrica solo existe en k de las n corridas; la media es sobre esas k.
Media ± desviación estándar muestral de 3 corridas por caso; el caso cuenta como completado solo si lo fueron todas sus corridas.
