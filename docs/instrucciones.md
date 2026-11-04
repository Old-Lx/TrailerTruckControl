# Instrucciones: entorno y ejecución del proyecto

Guía paso a paso para Windows (PowerShell). Todos los comandos se ejecutan desde la **raíz del repositorio**
(`...\Tésis\TrailerTruckControl`) salvo que se indique otra carpeta.

---

## 1. Requisitos

| Requisito | Versión | Nota |
|---|---|---|
| BeamNG.tech | **0.38.3** | Con licencia de investigación. Debe haberse abierto al menos una vez a mano. |
| Python | **3.11** | El proyecto se probó con 3.11.5 (las versiones de `requirements.txt`). |
| BeamNGpy | 1.35 | La instala `requirements.txt`. BeamNG.tech 0.38 ↔ BeamNGpy 1.35.x. |

Para ver qué versiones de Python hay instaladas: `py -0p`. Debe aparecer `-V:3.11`.

> El comando `python` a secas puede abrir **Python 3.12** (el primero en el PATH), que no tiene BeamNGpy.
> Por eso se trabaja siempre dentro del entorno virtual `.venv`, que usa 3.11.

---

## 2. Entorno virtual (`.venv`)

### 2.1 Crearlo (solo la primera vez; si la carpeta `.venv` ya existe, saltar al 2.2)
```powershell
py -3.11 -m venv .venv
```

### 2.2 Activarlo (cada vez que se abre una terminal nueva)
```powershell
.\.venv\Scripts\Activate.ps1
```
Al activarse, la línea de comandos empieza con `(.venv)`. A partir de ahí, `python` y `pip` son los del entorno.

Si PowerShell responde *"la ejecución de scripts está deshabilitada en este sistema"*, ejecutar **una sola vez**:
```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```
y volver a activar.

Alternativa sin activar: llamar al intérprete directamente, `.\.venv\Scripts\python.exe src\main.py`.

### 2.3 Instalar las dependencias (la primera vez o si cambia `requirements.txt`)
```powershell
python -m pip install -r requirements.txt
```

### 2.4 Comprobar
```powershell
python --version                      # Python 3.11.x
python -c "import beamngpy, numpy, scipy, cv2, pandas, matplotlib, dotenv; print('OK')"
```

### 2.5 Desactivar
```powershell
deactivate
```

### 2.6 VS Code
`Ctrl+Shift+P` → **Python: Select Interpreter** → elegir `.\.venv\Scripts\python.exe` de `TrailerTruckControl`.
Las terminales nuevas de VS Code activarán el entorno solas.

---

## 3. Configurar la ruta de BeamNG (`.env`)

El programa lee la instalación de BeamNG desde un archivo `.env` en la raíz del repo (no se sube a GitHub).
```powershell
Copy-Item .env.example .env
notepad .env
```
Contenido (ajustar las rutas a la instalación local):
```
BNG_HOME='C:\Users\<usuario>\BeamNG.tech.v0.38.3.0'
BNG_USER='C:\Users\<usuario>\BeamNG.tech.v0.38.3.0\<carpeta_de_usuario>'
```
- `BNG_HOME`: carpeta donde está `BeamNG.tech.exe`.
- `BNG_USER`: carpeta de datos de usuario de BeamNG (mods, configuraciones, caché).

El `.env` se encuentra aunque el programa se ejecute desde una subcarpeta (se busca hacia arriba desde `src/`).

---

## 4. Ejecutar un experimento

### 4.1 Elegir qué correr: `src/main.py`
```python
truck_trailer.TIPO_RUTA = 0      # 0 recta_x | 1 recta_y | 2 recta_diagonal | 3 parabola_x | 4 parabola_y | 5 seno_x | 6 seno_y
ALGORITMO = 'lqr'                # 'forward' | 'backward' | 'lqr'

fwd.LEY_DIRECCION = 'proporcional'          # marcha adelante: 'proporcional' | 'pure_pursuit'
bwd_lqr.PREALIMENTACION_CURVATURA = True    # LQR
bwd_lqr.PROGRAMACION_EN_V = True            # LQR
bwd_lqr.MODO_INTEGRADOR = 'corregido'       # LQR: 'corregido' (por defecto) | 'original'
```
Las rutas 1, 2 y 4 empiezan desalineadas con el vehículo a propósito (prueba de captura).

### 4.2 Correr
Los archivos de resultados se escriben **en la carpeta desde la que se ejecuta**. Para no mezclar corridas,
usar una carpeta por grupo de corridas dentro de `corridas/` (git la ignora; lo que vaya a la tesis se copia a `resultados/`):
```powershell
mkdir corridas\prueba ; cd corridas\prueba
python ..\..\src\main.py
cd ..\..
```

### 4.3 Qué pasa durante la corrida
1. Se abre BeamNG.tech (tarda ~1 min), se carga el mapa `smallgrid`, aparecen camión y tráiler y se acoplan.
2. En la consola se imprime el reporte de calibración de cámaras y el modo (síncrono: telemetría a 60 Hz, control a 7.5 Hz).
   Antes de la primera muestra el conjunto espera quieto con freno de mano (`[experimento] conjunto en reposo tras ... s`;
   `truck_trailer.ESPERAR_REPOSO`), para que todas las corridas partan del mismo estado.
   Ritmo: con la telemetría a 60 Hz la simulación va a ~0.32 veces el tiempo real (al terminar se reanuda y va a 1x,
   porque ya no hay peticiones por cuadro). `truck_trailer.DECIMACION_TELEMETRIA` = 4 (15 Hz) u 8 (7.5 Hz) la lleva a
   0.62x / 0.73x con la misma física; solo hay menos filas en el CSV.
3. Se abre la ventana de la cámara (frontal en marcha adelante, trasera en reversa) con la ruta en verde.
   En el mundo 3D la ruta se ve en magenta.
4. Al terminar (fin de ruta, jackknife o 300 s) se muestra la gráfica de respuesta y la consola pide
   `Presione enter...`. En marcha adelante BeamNG se cierra solo; en reversa queda abierto (cerrarlo a mano o reutilizarlo).

### 4.4 Archivos que produce
| Archivo | Contenido |
|---|---|
| `telemetria_<algoritmo>_<ruta>.csv` | una fila por paso de simulación (60 Hz) |
| `telemetria_<algoritmo>_<ruta>.json` | resultado, opciones usadas, sobrepaso, distancia final, duración |
| `ruta_<ruta>.csv` | la ruta de referencia |
| `respuesta_sistema.png` | gráfica rápida de la corrida |

> El nombre no incluye las opciones: para comparar variantes (p. ej. LQR con y sin prealimentación)
> usar carpetas distintas. Las opciones de cada corrida quedan en su `.json`.

---

## 5. Evaluar resultados (métricas, tablas y figuras del Capítulo 5)
```powershell
python src\evaluacion_resultados.py --datos corridas\prueba --salida corridas\prueba\out
```
Genera `metricas.csv`, `tabla_metricas.md`, `tabla_metricas.tex` y las figuras `trayectoria_*_xy` y
`respuestas_temporales_comparativas_*` (PNG y PDF). Opciones: `--formatos png pdf svg`, `--dpi 300`,
`--unidad-articulacion rad`.

**Repeticiones.** Las corridas de BeamNG no se repiten exactamente, así que cada caso se corre varias veces,
una carpeta por repetición, y se evalúan juntas:
```powershell
python src\evaluacion_resultados.py --datos corridas\campana_r1 corridas\campana_r2 corridas\campana_r3 --salida corridas\campana_out
```
Además de `metricas.csv` (una fila por corrida, con su carpeta) escribe `metricas_agregadas.csv` (media,
desviación estándar y número de valores por caso), y las tablas muestran media ± desviación. Las figuras se
dibujan con la primera carpeta.

Los resultados que se usen en la tesis se copian a `resultados/` (sí se versiona).

---

## 6. Identificar la planta (Capítulo 3)
```powershell
mkdir corridas\identificacion ; cd corridas\identificacion
python ..\..\src\experimento_identificacion.py          # ~153 s de simulación, marcha adelante, volante automático
cd ..\..
python src\identificar_planta.py corridas\identificacion\identificacion_planta.csv `
       --salida corridas\identificacion\out `
       --validar corridas\<carpeta>\telemetria_lqr_recta_y.csv     # opcional: una corrida en reversa
```
Escribe `parametros_identificados.json` y las figuras `identificacion_*`.

---

## 7. Verificar el software sin BeamNG (arnés)
Solo para comprobar que el código no se rompió tras un cambio. **Sus resultados son sintéticos y no son
resultados de la tesis.**
```powershell
mkdir corridas\arnes ; cd corridas\arnes
python ..\..\src\arx_truck_ai\arnes.py lqr 3                                 # algoritmo y ruta
python ..\..\src\arx_truck_ai\arnes.py lqr 5 PREALIMENTACION_CURVATURA=False # con una opción cambiada
python ..\..\src\arx_truck_ai\arnes.py identificacion 0                      # experimento de identificación
cd ..\..
```

---

## 8. Problemas frecuentes

| Síntoma | Causa | Solución |
|---|---|---|
| `ModuleNotFoundError: No module named 'beamngpy'` | Se está usando el Python global (3.12) | Activar el `.venv` (§2.2) o usar `.\.venv\Scripts\python.exe` |
| "la ejecución de scripts está deshabilitada" | Política de PowerShell | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
| BeamNG no abre o error de rutas | `.env` ausente o rutas incorrectas | Revisar §3 |
| Error de conexión al puerto 25252 | Ya hay un BeamNG abierto por otra corrida | Cerrar BeamNG y volver a correr |
| La columna `volante_real_deg` del CSV viene vacía | El sensor Electrics no se adjuntó | Revisar el aviso al inicio de la consola y avisar |
| El camión arranca hacia atrás solo al terminar | (corregido) freno de servicio con el vehículo detenido | La parada ya aplica freno de mano y neutro |
