from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Lidar, State
try:                                   # sensor de tiempo de simulación (no existe en todas las versiones)
    from beamngpy.sensors import Timer
except ImportError:
    Timer = None
import cv2
import numpy as np
import time
import math

# =====================================================================================
# CONFIGURACIÓN DEL EXPERIMENTO (Capítulo 5)
# =====================================================================================
# Ruta a recorrer. Se puede cambiar aquí o desde main.py ANTES de llamar a <algoritmo>.main():
#     from arx_truck_ai import truck_trailer
#     truck_trailer.TIPO_RUTA = 5
TIPO_RUTA = 0
# Nombre de cada ruta: lo usan los archivos de telemetría (telemetria_<algoritmo>_<nombre>.csv)
# y evaluacion_resultados.py para agrupar las figuras del Capítulo 5.
NOMBRES_RUTA = {
    0: 'recta_x',          # recta paralela al eje X
    1: 'recta_y',          # recta paralela al eje Y
    2: 'recta_diagonal',   # recta x = y
    3: 'parabola_x',       # y = a*x^2 (lado positivo)
    4: 'parabola_y',       # x = a*y^2 (lado positivo)
    5: 'seno_x',           # y = A*sin(w*x), avanza hacia X
    6: 'seno_y',           # x = A*sin(w*y), avanza hacia Y
}

# Ejecución síncrona: la simulación se mantiene en pausa y avanza exactamente un paso de 1/60 s por
# iteración (bng.step), de modo que la telemetría se registra a 60 Hz con tiempo de SIMULACIÓN.
# El control se recalcula cada DECIMACION_CONTROL pasos (60/8 = 7.5 Hz, el régimen validado en BeamNG;
# a 60 Hz los términos derivativos amplifican el ruido de los sensores y el volante vibra) y el comando
# se mantiene entre actualizaciones. Con MODO_SINCRONO = False se vuelve al modo anterior: simulación
# libre en tiempo real, una muestra por iteración del lazo (~6-7 Hz).
MODO_SINCRONO = True
FRECUENCIA_SIMULACION = 60     # Hz (bng.settings.set_deterministic)
DECIMACION_CONTROL = 8         # pasos de simulación por actualización del control (solo modo síncrono)
DURACION_MAXIMA_S = 300.0      # s de simulación: corte de seguridad si el experimento no termina

# =====================================================================================
# PROYECCIÓN DE LA RUTA EN LAS CÁMARAS Y PUNTO DE CONTROL
# =====================================================================================
# En el primer read_sensors() se ejecuta TruckTrailer.calibrar_geometria(), que MIDE en el
# simulador (una sola vez):
#   - la pose real de cada cámara respecto de su vehículo (Camera.get_position / get_direction),
#   - el centro lateral del camión y del tráiler respecto de state['pos'] (Vehicle.get_bbox),
#   - la altura del suelo (parte baja de la caja envolvente).
# Con eso la línea se dibuja donde realmente está la ruta y el punto de control queda en el
# eje longitudinal de cada vehículo. Antes, la cámara se reconstruía desde state['pos'] crudo
# mientras el control usaba ese punto corrido 0.949 m al centro: con el tráiler centrado sobre
# la ruta, la línea aparecía ~0.95 m al costado, sobre la línea de ruedas.
# Si una medición no está disponible en tu versión de BeamNGpy se usa el valor por defecto de
# abajo y se avisa en consola. El reporte de calibración se imprime al inicio de cada corrida.
OFFSET_LATERAL_TRUCK_DEFECTO = 0.0       # m (+ = hacia la izquierda del vehículo) del eje respecto de state['pos']
OFFSET_LATERAL_TRAILER_DEFECTO = 0.949   # m  valor que se usaba fijo hasta ahora en read_sensors()
OFFSET_LATERAL_MAXIMO = 1.5              # m  una medición mayor se descarta por inverosímil
FOV_SCALE = 0.98                         # ajuste fino de focal (ver verificación en el reporte de calibración)
DIBUJAR_RUTA_EN_MUNDO = True             # además, traza la ruta en el mundo 3D de BeamNG (referencia visual)
COLOR_RUTA_MUNDO = (1.0, 0.0, 1.0, 0.8)  # RGBA de esa línea (magenta, para distinguirla del overlay verde)
DIBUJAR_EJE_VEHICULO = True              # dibuja en la cámara el eje longitudinal del vehículo (línea blanca fina):
                                         # con el vehículo centrado sobre la ruta, verde y blanca coinciden

# Configuración de las cámaras (la usan set_sensors() y la reconstrucción de respaldo).
# Espacio del vehículo de BeamNG: -Y = adelante, +X = izquierda, +Z = arriba.
CAMARAS = {
    'front': dict(pos=(0, -0.216, 2.784), dir=(0, -0.965, -0.259), up=(0, 0, 1), fov_y=70,
                  y_pivot_offset=-3.0,   # solo para el respaldo (calibración a ojo anterior)
                  ventana="Video en Streaming - Route Cam (Front)"),
    'reverse': dict(pos=(0, 9.148, 3.960), dir=(0, 1, 0), up=(0, 0, 1), fov_y=70,
                    y_pivot_offset=0.0,
                    ventana="Video en Streaming - Reverse Cam (Trailer)"),
}


def _unit(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _marco_vehiculo(state: dict):
    """Origen (state['pos']) y matriz R cuyas columnas son (adelante, izquierda, arriba) del vehículo en el mundo."""
    p = np.asarray(state['pos'], dtype=np.float64)
    f = _unit(state['dir'])
    u0 = _unit(state['up'])
    l = _unit(np.cross(u0, f))
    u = np.cross(f, l)
    return p, np.column_stack((f, l, u))


def _extrinsecas_opencv(cam_pos_w, cam_dir_w, cam_up_w):
    """(rvec, tvec) de OpenCV para una cámara con posición, dirección de vista y 'arriba' en el mundo.
    Ejes de OpenCV: x = derecha de la imagen, y = abajo, z = dirección de vista."""
    z = _unit(cam_dir_w)
    x = _unit(np.cross(z, np.asarray(cam_up_w, dtype=np.float64)))
    y = np.cross(z, x)
    R_wc = np.vstack((x, y, z))
    rvec, _ = cv2.Rodrigues(R_wc)
    tvec = (-R_wc @ np.asarray(cam_pos_w, dtype=np.float64)).reshape(3, 1)
    return rvec, tvec

# Construye la matriz intrínseca K de la cámara para usar en cv2.projectPoints.
# Permite escalar la focal (fov_scale) para calibrar empíricamente si el render
# interno aplica un recorte/aspecto distinto al asumido.
def get_camera_intrinsics(fov_y_deg: float, width: int, height: int, fov_scale: float = 1.0) -> np.ndarray:
    fov_y_rad = math.radians(fov_y_deg)
    f_y = (height / 2.0) / math.tan(fov_y_rad / 2.0)
    f_x = f_y  # BeamNG usa píxeles cuadrados

    # Ajuste empírico opcional de focal (p.ej. 0.98–1.05) para corregir leve mismatch de FOV.
    f_x *= fov_scale
    f_y *= fov_scale

    c_x = width / 2.0
    c_y = height / 2.0

    K = np.array([
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0, 0, 1]
    ], dtype=np.float32)
    return K


def _build_T_vc(cam_pos: tuple, cam_dir: tuple, cam_up: tuple) -> np.ndarray:
    """Transformación homogénea cámara-en-vehículo.

    Usa los ejes locales de la cámara definidos en el propio sensor (`dir`, `up`).
    En BeamNG, `dir` apunta al frente de la cámara y `up` al techo del vehículo.
    """

    z_cam = np.array(cam_dir, dtype=np.float32)
    z_cam /= np.linalg.norm(z_cam)

    y_cam = np.array(cam_up, dtype=np.float32)
    y_cam /= np.linalg.norm(y_cam)

    x_cam = np.cross(y_cam, z_cam)
    x_cam /= np.linalg.norm(x_cam)

    # Re-ortonormalizamos Y para evitar drift numérico
    y_cam = np.cross(z_cam, x_cam)
    y_cam /= np.linalg.norm(y_cam)

    T_vc = np.eye(4, dtype=np.float32)
    T_vc[:3, :3] = np.column_stack((x_cam, y_cam, z_cam))
    T_vc[:3, 3] = np.array(cam_pos, dtype=np.float32)
    return T_vc


def _build_T_wv(truck_state: dict) -> np.ndarray:
    """Transformación homogénea vehículo-en-mundo usando `state` de BeamNG.

    Tomamos la convención más simple: ejes locales del camión son
    +X derecha, +Y frente, +Z arriba. `state['dir']` apunta al frente real del
    vehículo en coordenadas globales, `state['up']` al techo; reconstruimos el eje
    derecho con un cruzado. R_wv queda con columnas (right, forward, up).
    """

    forward_w = np.array(truck_state['dir'], dtype=np.float32)
    forward_w /= np.linalg.norm(forward_w)

    up_w = np.array(truck_state['up'], dtype=np.float32)
    up_w /= np.linalg.norm(up_w)

    right_w = np.cross(forward_w, up_w)
    right_w /= np.linalg.norm(right_w)

    # Re-ortonormaliza por estabilidad numérica
    up_w = np.cross(right_w, forward_w)
    up_w /= np.linalg.norm(up_w)

    T_wv = np.eye(4, dtype=np.float32)
    T_wv[:3, :3] = np.column_stack((right_w, forward_w, up_w))
    T_wv[:3, 3] = np.array(truck_state['pos'], dtype=np.float32)
    return T_wv


def get_camera_extrinsics(truck_state: dict, cam_pos: tuple, cam_dir: tuple, cam_up: tuple, y_pivot_offset: float = 0.0):
    """Devuelve (rvec, tvec) usando la convención real de BeamNG para el vehículo.

    Convención local del vehículo en BeamNG: +X = izquierda, +Y = atrás, +Z = arriba.
    Por tanto, el frente (hacia donde mira) es -Y local. Reconstruimos la rotación
    vehículo→mundo con columnas (-r, -f, u) para que cuando el camión mira a -Y,
    R_veh sea identidad. Luego proyectamos a OpenCV (eje y hacia abajo).
    """

    truck_p = np.array(truck_state['pos'], dtype=np.float32)
    truck_f = np.array(truck_state['dir'], dtype=np.float32)  # Frente global (-Y local)
    truck_u = np.array(truck_state['up'], dtype=np.float32)   # Techo global (+Z local)

    truck_f /= np.linalg.norm(truck_f)
    truck_u /= np.linalg.norm(truck_u)

    # Gram-Schmidt corregido: mantenemos el frente (yaw) como referencia "sagrada"
    # y ajustamos solo up a partir de right para no robar yaw cuando el chasis se inclina.
    truck_r = np.cross(truck_f, truck_u)
    if np.linalg.norm(truck_r) == 0:
        truck_r = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    else:
        truck_r /= np.linalg.norm(truck_r)

    # Recalculamos up con right x front (mano derecha) para ortonormalizar sin mover el yaw.
    truck_u = np.cross(truck_r, truck_f)
    truck_u /= np.linalg.norm(truck_u)

    # Rotación local→mundo del vehículo
    # Local BeamNG (con Camera): +X = derecha, +Y = frente (mira hacia -Y global), +Z = arriba.
    # Por eso usamos columnas (right, -front, up). Ortonormal y sin sesgo lateral.
    R_veh = np.column_stack((truck_r, -truck_f, truck_u))

    # Compensación de pivote (por si state['pos'] está atrasado respecto al morro).
    # El valor se pasa como parámetro (metros) para calibrar sin tocar código.
    cam_pos_adjusted = (cam_pos[0], cam_pos[1] + y_pivot_offset, cam_pos[2])

    # Posición y orientación de la cámara en mundo (BeamNG)
    cam_pos_world = truck_p + R_veh.dot(np.array(cam_pos_adjusted, dtype=np.float32))
    cam_dir_world = R_veh.dot(np.array(cam_dir, dtype=np.float32))
    cam_up_world = R_veh.dot(np.array(cam_up, dtype=np.float32))

    cam_dir_world /= np.linalg.norm(cam_dir_world)
    cam_up_world /= np.linalg.norm(cam_up_world)

    # Ejes ortonormales de la cámara
    z_cam = cam_dir_world
    y_cam = -cam_up_world  # OpenCV tiene Y hacia abajo en imagen
    x_cam = np.cross(y_cam, z_cam)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    y_cam /= np.linalg.norm(y_cam)

    R_cw = np.column_stack((x_cam, y_cam, z_cam))
    R_wc = R_cw.T

    tvec = -R_wc @ cam_pos_world
    rvec, _ = cv2.Rodrigues(R_wc)

    return rvec, tvec


def _extract_cam_pose_from_data(cam_data: dict):
    """Intenta leer pose absoluta de la cámara desde el `poll` del sensor.

    Algunos modos de BeamNGpy entregan `pos`, `dir` y `up` en el diccionario de
    la cámara. Si no están presentes, devolvemos `None` para seguir usando la
    composición vehículo + offset.
    """

    if not isinstance(cam_data, dict):
        return None

    pos = cam_data.get('pos') or cam_data.get('position')
    dir_vec = cam_data.get('dir') or cam_data.get('direction')
    up_vec = cam_data.get('up')

    if pos is None or dir_vec is None or up_vec is None:
        return None

    return (
        np.array(pos, dtype=np.float32),
        np.array(dir_vec, dtype=np.float32),
        np.array(up_vec, dtype=np.float32),
    )


def get_camera_extrinsics_from_world_pose(cam_pos_w: np.ndarray, cam_dir_w: np.ndarray, cam_up_w: np.ndarray):
    """Devuelve (rvec, tvec) a partir de la pose de la cámara en el mundo.
    (La versión anterior armaba la base de la cámara con un eje x invertido y proyectaba mal;
    no se notaba porque no se usaba.)"""
    if np.linalg.norm(cam_dir_w) == 0 or np.linalg.norm(cam_up_w) == 0:
        return None
    return _extrinsecas_opencv(cam_pos_w, cam_dir_w, cam_up_w)

def _rotate_and_translate_route(points: np.ndarray, origin: tuple[float, float, float], yaw: float) -> list:
    """Rota los puntos en XY por `yaw` y luego los traslada al `origin`."""

    if points is None or len(points) == 0:
        return []

    pts = np.asarray(points, dtype=np.float32)

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    rot = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=np.float32)

    xy_rotated = pts[:, :2] @ rot.T

    translated = np.empty_like(pts)
    translated[:, 0] = xy_rotated[:, 0] + origin[0]
    translated[:, 1] = xy_rotated[:, 1] + origin[1]
    translated[:, 2] = pts[:, 2] + origin[2]

    return translated.tolist()


def gen_line_x(length: float = 200.0, step: float = 1.0, z: float = 0.0) -> np.ndarray:
    """Línea recta paralela al eje X partiendo de (0, 0, z)."""

    xs = np.arange(0.0, length + step, step, dtype=np.float32)
    ys = np.zeros_like(xs)
    zs = np.full_like(xs, z, dtype=np.float32)
    return np.column_stack((xs, ys, zs))


def gen_line_y(length: float = 200.0, step: float = 1.0, z: float = 0.0) -> np.ndarray:
    """Línea recta paralela al eje Y partiendo de (0, 0, z)."""

    ys = np.arange(0.0, length + step, step, dtype=np.float32)
    xs = np.zeros_like(ys)
    zs = np.full_like(ys, z, dtype=np.float32)
    return np.column_stack((xs, ys, zs))


def gen_line_diagonal(length: float = 200.0, step: float = 1.0, z: float = 0.0) -> np.ndarray:
    """Línea recta con pendiente 1 (x = y) en el plano XY."""

    t = np.arange(0.0, length + step, step, dtype=np.float32)
    zs = np.full_like(t, z, dtype=np.float32)
    return np.column_stack((t, t, zs))


def gen_parabola_y_from_x(a: float = 0.01, x_max: float = 80.0, step: float = 0.5, z: float = 0.0) -> np.ndarray:
    """Parábola de la forma y = a * x^2."""

    xs = np.arange(0.0, x_max + step, step, dtype=np.float32)
    ys = a * xs ** 2
    zs = np.full_like(xs, z, dtype=np.float32)
    return np.column_stack((xs, ys, zs))


def gen_parabola_x_from_y(a: float = 0.01, y_max: float = 80.0, step: float = 0.5, z: float = 0.0) -> np.ndarray:
    """Parábola de la forma x = a * y^2."""

    ys = np.arange(0.0, y_max + step, step, dtype=np.float32)
    xs = a * ys ** 2
    zs = np.full_like(ys, z, dtype=np.float32)
    return np.column_stack((xs, ys, zs))


def gen_sine_x(amplitude: float = 3.0, wavelength: float = 40.0, length: float = 200.0, step: float = 1.0, phase: float = 0.0, z: float = 0.0) -> np.ndarray:
    """Onda senoidal oscilando en Y mientras avanza en X."""

    xs = np.arange(0.0, length + step, step, dtype=np.float32)
    ys = amplitude * np.sin((2 * np.pi / wavelength) * xs + phase)
    zs = np.full_like(xs, z, dtype=np.float32)
    return np.column_stack((xs, ys, zs))


def gen_sine_y(amplitude: float = 3.0, wavelength: float = 40.0, length: float = 200.0, step: float = 1.0, phase: float = 0.0, z: float = 0.0) -> np.ndarray:
    """Onda senoidal oscilando en X mientras avanza en Y."""

    ys = np.arange(0.0, length + step, step, dtype=np.float32)
    xs = amplitude * np.sin((2 * np.pi / wavelength) * ys + phase)
    zs = np.full_like(ys, z, dtype=np.float32)
    return np.column_stack((xs, ys, zs))


# Tipos soportados en make_route:
# 0: Línea en X | 1: Línea en Y | 2: Diagonal | 3: y = a*x^2 | 4: x = a*y^2 | 5: Seno en X | 6: Seno en Y
def make_route(origin: tuple[float, float, float], route_type: int, yaw: float = 0.0, **params) -> list:
    """Genera la ruta solicitada, la rota por `yaw` y la traslada a `origin`."""

    z = params.get('z', 0.0)

    match route_type:
        case 0:
            base = gen_line_x(length=params.get('length', 200.0), step=params.get('step', 1.0), z=z)
        case 1:
            base = gen_line_y(length=params.get('length', 200.0), step=params.get('step', 1.0), z=z)
        case 2:
            base = gen_line_diagonal(length=params.get('length', 200.0), step=params.get('step', 1.0), z=z)
        case 3:
            base = gen_parabola_y_from_x(a=params.get('a', 0.01), x_max=params.get('span', 80.0), step=params.get('step', 0.5), z=z)
        case 4:
            base = gen_parabola_x_from_y(a=params.get('a', 0.01), y_max=params.get('span', 80.0), step=params.get('step', 0.5), z=z)
        case 5:
            base = gen_sine_x(amplitude=params.get('amplitude', 3.0), wavelength=params.get('wavelength', 40.0), length=params.get('length', 200.0), step=params.get('step', 1.0), phase=params.get('phase', 0.0), z=z)
        case 6:
            base = gen_sine_y(amplitude=params.get('amplitude', 3.0), wavelength=params.get('wavelength', 40.0), length=params.get('length', 200.0), step=params.get('step', 1.0), phase=params.get('phase', 0.0), z=z)
        case _:
            base = np.empty((0, 3), dtype=np.float32)

    return _rotate_and_translate_route(base, origin, yaw)

# Rutina para mostrar la imagen de la cámara
def stream_cam(cam_data: dict, route: list, veh_state: dict, is_reverse: bool = False,
               pose_camara=None, z_ruta=None, eje_vehiculo=None):
    """Proyecta la ruta (coords mundo) a la imagen de la cámara (front_cam o reverse_cam).

    pose_camara: (pos, dir, up) de la cámara en el mundo, calculada por TruckTrailer a partir de la
                 calibración; si es None se usa la reconstrucción anterior (solo por compatibilidad).
    z_ruta:      altura a la que se dibuja la ruta (el suelo medido); si es None, la z de la ruta.
    eje_vehiculo: puntos (mundo) del eje longitudinal del vehículo sobre el suelo, para dibujarlo como
                 referencia (línea blanca fina); None para no dibujarlo.
    """

    # Extraer la imagen a color
    if 'colour' in cam_data:
        # BeamNGpy suele entregar una imagen de formato PIL RGBA
        img_pil = cam_data['colour']

        # Convertir de PIL Image a un Array de Numpy (para OpenCV)
        img_array = np.array(img_pil)

        # Convertir el formato de color de RGBA (BeamNG) a BGR (estándar OpenCV)
        if img_array.shape[2] == 4:  # Si tiene canal Alpha (transparencia)
            img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
        else:
            img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

        cfg = CAMARAS['reverse' if is_reverse else 'front']
        window_name = cfg['ventana']

        extrinsics = None
        if pose_camara is not None:
            # Pose calibrada (ver TruckTrailer.calibrar_geometria / pose_camara_mundo)
            extrinsics = _extrinsecas_opencv(*pose_camara)
        else:
            cam_pose = _extract_cam_pose_from_data(cam_data)
            if cam_pose is not None:
                extrinsics = get_camera_extrinsics_from_world_pose(*cam_pose)
        if extrinsics is None:
            # Reconstrucción anterior (sin calibrar): desde state['pos'] crudo + offsets de configuración.
            rvec, tvec = get_camera_extrinsics(veh_state, cfg['pos'], cfg['dir'], cfg['up'], y_pivot_offset=cfg['y_pivot_offset'])
        else:
            rvec, tvec = extrinsics

        # Intrínseca de la cámara de BeamNG (512x512) con escala ajustable de focal
        alto, ancho = img_bgr.shape[:2]
        K = get_camera_intrinsics(fov_y_deg=cfg['fov_y'], width=ancho, height=alto, fov_scale=FOV_SCALE)

        route_np = np.array(route, dtype=np.float32)
        if z_ruta is not None and len(route_np):
            route_np = route_np.copy()
            route_np[:, 2] = z_ruta      # la ruta va sobre el suelo, no a la altura de spawn del vehículo

        # Filtramos puntos detrás de la cámara (z<=0) o demasiado lejos para evitar “saltos”.
        R_wc, _ = cv2.Rodrigues(rvec)
        route_cam = (R_wc @ route_np.T) + tvec.reshape(3, 1)
        depth_mask = (route_cam[2, :] > 0.5) & (route_cam[2, :] < 500.0)

        if np.any(depth_mask):
            route_visible = route_np[depth_mask]
            image_points, _ = cv2.projectPoints(route_visible, rvec, tvec, K, np.zeros((4, 1)))

            if image_points is not None and len(image_points) > 1:
                puntos_pantalla = np.int32(image_points).reshape(-1, 2)

                # Limitar a un rango amplio para no dibujar puntos basura fuera de cuadro
                puntos_validos = [p for p in puntos_pantalla if -2000 < p[0] < 3000 and -2000 < p[1] < 3000]

                if len(puntos_validos) > 1:
                    puntos_np = np.array(puntos_validos).reshape((-1, 1, 2))
                    cv2.polylines(img_bgr, [puntos_np], isClosed=False, color=(0, 255, 0), thickness=3)

        # Eje longitudinal del vehículo proyectado con la misma cámara (línea blanca fina)
        if eje_vehiculo is not None and len(eje_vehiculo) > 1:
            eje_np = np.asarray(eje_vehiculo, dtype=np.float32)
            prof = ((R_wc @ eje_np.T) + tvec.reshape(3, 1))[2, :]
            eje_np = eje_np[(prof > 0.5) & (prof < 500.0)]
            if len(eje_np) > 1:
                pe, _ = cv2.projectPoints(eje_np, rvec, tvec, K, np.zeros((4, 1)))
                pe = np.int32(pe).reshape(-1, 2)
                pe = pe[(pe[:, 0] > -2000) & (pe[:, 0] < 3000) & (pe[:, 1] > -2000) & (pe[:, 1] < 3000)]
                if len(pe) > 1:
                    cv2.polylines(img_bgr, [pe.reshape((-1, 1, 2))], isClosed=False, color=(255, 255, 255), thickness=1)

        # Visualizar el streaming en una ventana emergente
        cv2.imshow(window_name, img_bgr)

        # OpenCV necesita esta pequeña pausa (1 milisegundo) para poder dibujar la ventana
        cv2.waitKey(1)

# Clase Camión con Trailer
class TruckTrailer:
    # Sensores
    imu_truck: AdvancedIMU
    imu_trailer: AdvancedIMU
    lidar_front: Lidar
    lidar_rear: Lidar
    front_cam: Camera
    reverse_cam: Camera
    state_truck: State
    state_trailer: State

    route: list

    # Debo revisar sus coordenadas respecto al mapa ya que las que usan para creación son respecto al camión o al trailer
    truck_origin: tuple[float, float, float]
    trailer_origin: tuple[float, float, float]

    # Constructor
    def __init__(self, truck: Vehicle, trailer: Vehicle, bng: BeamNGpy):
        self.truck = truck
        self.trailer = trailer
        self.bng = bng
        self._geometria = None   # se llena en calibrar_geometria() en el primer read_sensors()
        self._pasos = 0          # pasos de simulación avanzados en modo síncrono
        self._t_reloj0 = time.time()
        self._fuente_tiempo = None

    # ------------------------------------------------------------------ ejecución del experimento
    @property
    def nombre_ruta(self):
        """Nombre de la ruta seleccionada (ver NOMBRES_RUTA)."""
        return NOMBRES_RUTA.get(TIPO_RUTA, f'ruta_{TIPO_RUTA}')

    @property
    def decimacion_control(self):
        """Iteraciones del lazo por cada actualización del control (1 en modo libre)."""
        return max(1, int(DECIMACION_CONTROL)) if MODO_SINCRONO else 1

    @property
    def dt_control(self):
        """Período nominal del control [s] en modo síncrono (None en modo libre)."""
        return self.decimacion_control / FRECUENCIA_SIMULACION if MODO_SINCRONO else None

    def iniciar_simulacion(self, gear=None):
        """Configura el paso determinista y deja la simulación lista para el experimento.
        En modo síncrono queda en pausa (avanza con avanzar()); en modo libre corre en tiempo real."""
        self.bng.control.pause()
        self.bng.settings.set_deterministic(FRECUENCIA_SIMULACION)
        if gear is not None:
            self.truck.control(gear=gear)
        if not MODO_SINCRONO:
            self.bng.control.resume()
        modo = (f"SÍNCRONO: registro a {FRECUENCIA_SIMULACION} Hz, control a "
                f"{FRECUENCIA_SIMULACION / self.decimacion_control:.1f} Hz" if MODO_SINCRONO else "LIBRE (tiempo real)")
        print(f"[experimento] ruta '{self.nombre_ruta}' (TIPO_RUTA={TIPO_RUTA}) | modo {modo}")

    def avanzar(self, pasos=1):
        """En modo síncrono avanza la simulación `pasos` pasos de 1/FRECUENCIA_SIMULACION s."""
        if not MODO_SINCRONO:
            return
        try:
            self.bng.control.step(pasos)       # BeamNGpy reciente
        except AttributeError:
            self.bng.step(pasos)               # BeamNGpy anterior
        self._pasos += pasos

    def liberar_simulacion(self):
        """Al terminar, deja la simulación corriendo para poder inspeccionarla (modo síncrono)."""
        if MODO_SINCRONO:
            try:
                self.bng.control.resume()
            except Exception:
                pass

    def tiempo_simulacion(self):
        """Tiempo [s]: el del sensor Timer de BeamNG si existe; si no, pasos/FRECUENCIA en modo
        síncrono o el reloj de pared en modo libre."""
        t = None
        if self._fuente_tiempo in (None, 'timer'):
            try:
                t = float(self.truck.sensors['timer']['time'])
                fuente = 'timer'
            except Exception:
                t = None
        if t is None:
            if MODO_SINCRONO:
                t, fuente = self._pasos / FRECUENCIA_SIMULACION, 'pasos'
            else:
                t, fuente = time.time() - self._t_reloj0, 'reloj'
        if self._fuente_tiempo != fuente:
            nombres = {'timer': 'sensor Timer de BeamNG', 'pasos': 'conteo de pasos de simulación',
                       'reloj': 'reloj de pared (sin sensor Timer)'}
            print(f"[experimento] tiempo de la telemetría: {nombres[fuente]}")
            self._fuente_tiempo = fuente
        return t

    # ------------------------------------------------------------------ calibración geométrica
    def _medir_offset_lateral(self, vehicle, p, R, defecto, nombre):
        """Desfase lateral (+ = izquierda) del eje longitudinal del vehículo respecto de state['pos'],
        medido con la caja envolvente. Devuelve (offset, info)."""
        try:
            bb = vehicle.get_bbox()
            pts = np.array([v for v in bb.values() if np.size(v) == 3], dtype=np.float64)
            if len(pts) < 8:
                raise ValueError(f"get_bbox() devolvió {len(pts)} esquinas")
            c = pts.mean(axis=0)
            lat = float((c - p) @ R[:, 1])
            info = dict(fuente='caja envolvente (get_bbox)', lon=float((c - p) @ R[:, 0]),
                        zmin=float(pts[:, 2].min()), largo=float(np.ptp(pts @ R[:, 0])), ancho=float(np.ptp(pts @ R[:, 1])))
            if abs(lat) > OFFSET_LATERAL_MAXIMO:
                info['fuente'] = f'valor por defecto (medición inverosímil: {lat:+.2f} m)'
                return defecto, info
            return lat, info
        except Exception as e:
            return defecto, dict(fuente=f'valor por defecto (get_bbox no disponible: {type(e).__name__})', zmin=None)

    def _pose_local_camara(self, cam, cfg, p, R, lat):
        """Pose de la cámara en el marco (adelante, izquierda, arriba) con origen state['pos'] de su vehículo.
        Primero intenta medirla en el simulador; si no se puede, usa la configuración del sensor
        suponiendo que la cámara está en el eje longitudinal del vehículo (x = 0)."""
        try:
            pos_w = np.asarray(cam.get_position(), dtype=np.float64)
            dir_w = _unit(cam.get_direction())
            if pos_w.shape != (3,) or dir_w.shape != (3,) or np.linalg.norm(pos_w - p) > 20.0:
                raise ValueError("pose inverosímil")
            up_w = _unit(R[:, 2] - (R[:, 2] @ dir_w) * dir_w)
            local = (R.T @ (pos_w - p), R.T @ dir_w, R.T @ up_w)
            fuente = 'MEDIDA en el simulador (Camera.get_position / get_direction)'
        except Exception as e:
            cx, cy, cz = cfg['pos']
            dx, dy, dz = cfg['dir']
            pos_l = np.array([-(cy + cfg['y_pivot_offset']), lat + cx, cz], dtype=np.float64)
            local = (pos_l, _unit([-dy, dx, dz]), np.array([0.0, 0.0, 1.0]))
            fuente = f'configuración del sensor, centrada en el eje (no se pudo medir: {type(e).__name__})'
        return local, fuente

    def calibrar_geometria(self, estado_camion, estado_trailer):
        """Mide (una vez) el centro lateral de cada vehículo, la altura del suelo y la pose de cada cámara."""
        p_t, R_t = _marco_vehiculo(estado_camion)
        p_r, R_r = _marco_vehiculo(estado_trailer)
        g = {}
        g['lat_truck'], info_t = self._medir_offset_lateral(self.truck, p_t, R_t, OFFSET_LATERAL_TRUCK_DEFECTO, 'camión')
        g['lat_trailer'], info_r = self._medir_offset_lateral(self.trailer, p_r, R_r, OFFSET_LATERAL_TRAILER_DEFECTO, 'tráiler')
        zs = [z for z in (info_t.get('zmin'), info_r.get('zmin')) if z is not None]
        g['z_suelo'] = min(zs) + 0.02 if zs else None        # 2 cm sobre el suelo para que la línea no se hunda

        g['cam'] = {}
        for nombre, cam, (p, R), lat in (('front', getattr(self, 'front_cam', None), (p_t, R_t), g['lat_truck']),
                                         ('reverse', getattr(self, 'reverse_cam', None), (p_r, R_r), g['lat_trailer'])):
            local, fuente = self._pose_local_camara(cam, CAMARAS[nombre], p, R, lat)
            # Pose que usaba la reconstrucción anterior (para mostrar cuánto se corría)
            cfg = CAMARAS[nombre]
            cx, cy, cz = cfg['pos']
            antes_l = np.array([-(cy + cfg['y_pivot_offset']), -cx, cz])  # anterior: +X local -> derecha
            g['cam'][nombre] = dict(local=local, fuente=fuente, delta=local[0] - antes_l)
        self._geometria = g

        print("\n[calibración de cámaras y punto de control] ------------------------------------------")
        for etq, lat, info in (('camión ', g['lat_truck'], info_t), ('tráiler', g['lat_trailer'], info_r)):
            extra = (f" | caja: {info['largo']:.2f} m de largo x {info['ancho']:.2f} m de ancho, centro {info['lon']:+.2f} m adelante de state['pos']"
                     if 'largo' in info else '')
            print(f"  {etq}: eje longitudinal a {lat:+.3f} m (+ = izquierda) de state['pos'] -> {info['fuente']}{extra}")
        print(f"  suelo  : la ruta se dibuja a z = {g['z_suelo']:.3f} m" if g['z_suelo'] is not None
              else "  suelo  : no medido; la ruta se dibuja a la z con que se generó")
        for nombre in ('front', 'reverse'):
            c = g['cam'][nombre]
            d = c['delta']
            print(f"  cámara {nombre:7s}: pose {c['fuente']}")
            print(f"           diferencia con la reconstrucción anterior: lateral {d[1]:+.3f} m, longitudinal {d[0]:+.3f} m, vertical {d[2]:+.3f} m")
        self._verificar_proyeccion(estado_camion, estado_trailer)
        print("--------------------------------------------------------------------------------------\n")
        self._dibujar_ruta_en_mundo()

    @staticmethod
    def _eje_vehiculo(punto_control, sentido, z_ruta, largo=60.0, paso=1.0):
        """Puntos del eje longitudinal del vehículo, sobre el suelo, desde el punto de control hacia `sentido`."""
        if not DIBUJAR_EJE_VEHICULO:
            return None
        base = np.asarray(punto_control, dtype=np.float64)
        d = np.asarray(sentido, dtype=np.float64).copy()
        d[2] = 0.0
        d = _unit(d)
        pts = base + np.outer(np.arange(0.0, largo, paso), d)
        if z_ruta is not None:
            pts[:, 2] = z_ruta
        return pts

    def pose_camara_mundo(self, nombre, estado):
        """Pose (pos, dir, up) de la cámara en el mundo para el estado actual de su vehículo."""
        p, R = _marco_vehiculo(estado)
        pos_l, dir_l, up_l = self._geometria['cam'][nombre]['local']
        return p + R @ pos_l, R @ dir_l, R @ up_l

    def _verificar_proyeccion(self, estado_camion, estado_trailer):
        """Comprueba la proyección contra la de BeamNG (Camera.world_point_to_pixel) en un punto de la ruta."""
        if not getattr(self, 'route', None):
            return
        for nombre, cam, estado in (('front', getattr(self, 'front_cam', None), estado_camion),
                                    ('reverse', getattr(self, 'reverse_cam', None), estado_trailer)):
            try:
                pos, d, up = self.pose_camara_mundo(nombre, estado)
                rvec, tvec = _extrinsecas_opencv(pos, d, up)
                K = get_camera_intrinsics(CAMARAS[nombre]['fov_y'], 512, 512, FOV_SCALE)
                pts = np.array(self.route, dtype=np.float64)
                if self._geometria['z_suelo'] is not None:
                    pts[:, 2] = self._geometria['z_suelo']
                R_wc, _ = cv2.Rodrigues(rvec)
                zc = (R_wc @ pts.T + tvec)[2]
                validos = np.where((zc > 3.0) & (zc < 60.0))[0]
                if not len(validos):
                    continue
                P = pts[validos[len(validos) // 2]]
                uv, _ = cv2.projectPoints(P.reshape(1, 3), rvec, tvec, K, np.zeros((4, 1)))
                ub = np.asarray(cam.world_point_to_pixel(tuple(P.tolist())), dtype=np.float64).ravel()
                print(f"  verificación {nombre:7s}: punto de la ruta a {zc[validos[len(validos) // 2]]:.1f} m -> "
                      f"píxel calculado ({uv.ravel()[0]:.0f}, {uv.ravel()[1]:.0f}) vs píxel de BeamNG ({ub[0]:.0f}, {ub[1]:.0f})")
            except Exception as e:
                print(f"  verificación {nombre:7s}: no disponible en esta versión de BeamNGpy ({type(e).__name__})")

    def _dibujar_ruta_en_mundo(self):
        """Traza la ruta en el mundo 3D de BeamNG (referencia para comprobar el overlay de las cámaras)."""
        if not DIBUJAR_RUTA_EN_MUNDO or not getattr(self, 'route', None):
            return
        z = self._geometria['z_suelo'] if self._geometria['z_suelo'] is not None else None
        coords = [(float(x), float(y), float(z if z is not None else pz)) for x, y, pz in self.route]
        intentos = (
            lambda: self.bng.debug.add_polyline(coords, COLOR_RUTA_MUNDO, cling=True, offset=0.05),   # BeamNGpy reciente
            lambda: self.bng.add_debug_polyline(coords, COLOR_RUTA_MUNDO, cling=True, offset=0.05),   # BeamNGpy anterior
        )
        for intento in intentos:
            try:
                intento()
                print("  ruta trazada también en el mundo 3D de BeamNG (línea magenta). "
                      "Desactívalo con DIBUJAR_RUTA_EN_MUNDO = False para las corridas finales.")
                return
            except Exception:
                continue
        print("  aviso: esta versión de BeamNGpy no permitió trazar la ruta en el mundo 3D (no afecta al control).")

    # Función para asignar los sensores
    def set_sensors(self):
        # Sensores del camión

        # Estado del camión
        self.state_truck = State()
        if Timer is not None:                       # tiempo de simulación para la telemetría
            try:
                self.truck.attach_sensor('timer', Timer())
            except Exception as e:
                print(f"[experimento] aviso: no se pudo adjuntar el sensor Timer ({type(e).__name__})")
        self.truck.attach_sensor('state_truck', self.state_truck)

        # IMU para medir posición y orientación del camión
        self.imu_truck = AdvancedIMU(
            "accel1", 
            self.bng, 
            self.truck,
            pos=(0, -0.394, 0.7),   # En el parachoque pero más arriba
            dir=(0, 1, 0),
            up=(0, 0, 1),
            gfx_update_time=0.0005,
            physics_update_time=0.0005,

            # Smoothing strength
            smoother_strength=1.0,

            # Sensor behavior flags
            is_using_gravity=True,
            is_visualised=True,
            is_snapping_desired=True,
            is_force_inside_triangle=False,
            is_allow_wheel_nodes=False
        )

        # Cámara para seguir la ruta
        self.front_cam = Camera(
            'front_cam', 
            self.bng, 
            vehicle=self.truck, 
            pos=CAMARAS['front']['pos'],   # En la parte del techo del camión en el frente
            dir=CAMARAS['front']['dir'],   # Viendo 15 grados hacia abajo del eje Y en el plano YZ
            up=CAMARAS['front']['up'],
            resolution=(512, 512),
            field_of_view_y=CAMARAS['front']['fov_y'], # 70 es el default
            is_using_shared_memory=True,
            is_visualised=True,
            is_streaming=True,
            is_snapping_desired=False,
            is_force_inside_triangle=False
        )

         # Sensor para los golpes marcha hacia delante
        self.lidar_front = Lidar(
            "lidar_front", 
            self.bng, 
            self.truck,
            requested_update_time=0.01, 
            is_using_shared_memory=True,
            is_360_mode=False,
            is_rotate_mode= False,
            horizontal_angle=90,
            pos=(0, -0.324, 0.320), # En la parte de abajo del parachoque
            dir=(0, -1, 0),        # Apuntando hacia adelante (Eje -Y en BeamNG)
            up=(0, 0, 1),
            vertical_resolution=16, 
            frequency=20, 
            is_visualised=False
        )

        # Sensores del trailer

        # Estado del trailer
        self.state_trailer = State()
        self.trailer.attach_sensor('state_trailer', self.state_trailer)

        # IMU para medir posición y orientación del trailer
        self.imu_trailer = AdvancedIMU(
            "imu_trailer", 
            self.bng, 
            self.trailer,
            pos=(0, 1.765, 0.877), 
            dir=(0, 1, 0), 
            up=(0, 0, 1), # En el kingpin
            gfx_update_time=0.0005, 
            physics_update_time=0.0005,
            smoother_strength=1.0, 
            is_using_gravity=True, 
            is_visualised=True
        )

        # Cámara de marcha hacia atrás
        self.reverse_cam = Camera(
            'reverse_cam', 
            self.bng, 
            vehicle=self.trailer,
            pos=CAMARAS['reverse']['pos'],   # En la parte de atrás y arriba del trailer
            dir=CAMARAS['reverse']['dir'],
            up=CAMARAS['reverse']['up'],
            resolution=(512, 512),
            field_of_view_y=CAMARAS['reverse']['fov_y'],
            is_using_shared_memory=True,
            is_visualised=True,
            is_streaming=True,
            is_snapping_desired=False,
            is_force_inside_triangle=False
        )

        # Sensor para los golpes marcha hacia atrás
        self.lidar_rear = Lidar(
            "lidar_rear", 
            self.bng, 
            self.trailer,
            requested_update_time=0.01,
              is_using_shared_memory=True,
            is_360_mode=False,
            is_rotate_mode= False,
            horizontal_angle=90,
            pos=(0, 9.150, 0.376), # En la parte de atrás del trailer y abajo
            dir=(0, 1, 0),         # Apuntando hacia atrás (Eje +Y en BeamNG)
            up=(0, 0, 1.0),
            vertical_resolution=16, 
            frequency=20, 
            is_visualised=False
        )

    # Función para leer los sensores del Truck Trailer
    def read_sensors(self, is_reverse=False, completo=True):
        """Lee el estado del conjunto. completo=False omite IMU, lidar y cámaras (no se usan en el
        control): es la lectura de los pasos intermedios del modo síncrono, solo para la telemetría."""
        # Leer datos de los sensores
        if completo:
            truck_imu_data = self.imu_truck.poll() # Estos dos son el estado físico del sistema
            trailer_imu_data = self.imu_trailer.poll()

            lidar_front_data = self.lidar_front.poll()
            lidar_rear_data = self.lidar_rear.poll()

        # Actualizar el estado físico de los vehículos (para leer el Yaw)
        self.truck.sensors.poll()
        self.trailer.sensors.poll()

        estado_camion = self.truck.state
        estado_trailer = self.trailer.state

        dir_truck = estado_camion['dir']
        dir_trailer = estado_trailer['dir']

        # truck_imu_data = data_truck['accel1']
        # trailer_imu_data = data_trailer['imu_trailer']

        # Extraer la velocidad longitudinal (v1) usando el vector de velocidad
        vel_truck = estado_camion['vel']
        v1 = np.linalg.norm(vel_truck) # Magnitud en m/s

        # Extraer las variables para el modelo cinemático
        # v1 = imu_data[0]['mass']
        
        psi_truck = np.arctan2(dir_truck[1], dir_truck[0])
        psi_trailer = np.arctan2(dir_trailer[1], dir_trailer[0])
        
        # --- CORRECCIÓN 1: NORMALIZACIÓN DE ARTICULACIÓN ---
        # Calculamos delta_F2 y aplicamos una restricción modular para que siempre
        # esté en el rango [-pi, pi]. Evita cálculos matemáticos erróneos de +-6.28 radianes 
        # que disparen los topes anti-jackknife al iniciar o en maniobras bruscas.
        delta_F2 = (psi_truck - psi_trailer + math.pi) % (2 * math.pi) - math.pi

        # Calibración geométrica (solo la primera vez): pose real de las cámaras y centro de cada vehículo
        if self._geometria is None:
            self.calibrar_geometria(estado_camion, estado_trailer)

        # --- CORRECCIÓN 2: PUNTO DE CONTROL EN EL EJE LONGITUDINAL DE CADA VEHÍCULO ---
        # state['pos'] no está en el eje de los vehículos: el desfase lateral se MIDE al inicio con la caja
        # envolvente (antes se usaba un 0.949 fijo solo para el tráiler). La posición a lo largo del
        # vehículo NO se cambia, para que siga valiendo el modelo identificado del LQR (DP_NOMINAL).
        p_c, R_c = _marco_vehiculo(estado_camion)
        p_r, R_r = _marco_vehiculo(estado_trailer)
        truck_pos_final = (p_c + R_c[:, 1] * self._geometria['lat_truck']).tolist()
        trailer_pos_final = (p_r + R_r[:, 1] * self._geometria['lat_trailer']).tolist()

        # Vectores del tráiler (se siguen usando para el roll)
        trailer_dir_vec = np.array(estado_trailer['dir'], dtype=np.float32)
        trailer_up_vec = np.array(estado_trailer['up'], dtype=np.float32)
        
        # Normalizamos los vectores de dirección
        trailer_dir_vec /= np.linalg.norm(trailer_dir_vec)
        trailer_up_vec /= np.linalg.norm(trailer_up_vec)
        
        # Obtenemos el vector de cara hacia la derecha del tráiler y lo normalizamos
        trailer_right_vec = np.cross(trailer_dir_vec, trailer_up_vec)
        trailer_right_vec /= np.linalg.norm(trailer_right_vec)


        # --- EXTRACCIÓN DE 8 GRADOS DE LIBERTAD (ROLL Y PITCH) ---
        # Dado que BeamNG aplica dinámicas de suspensión, el chasis puede balancearse (Roll).
        # El vector "right" del camión cruzando dir y up
        truck_dir_vec = np.array(dir_truck, dtype=np.float32)
        truck_up_vec = np.array(estado_camion['up'], dtype=np.float32)
        truck_dir_vec /= np.linalg.norm(truck_dir_vec)
        truck_up_vec /= np.linalg.norm(truck_up_vec)
        truck_right_vec = np.cross(truck_dir_vec, truck_up_vec)
        truck_right_vec /= np.linalg.norm(truck_right_vec)

        # Roll: ángulo del vector right respecto a la horizontal pura (inclinación en Z)
        # pitch: ángulo del vector dir respecto a la horizontal pura en Z
        truck_roll = math.asin(np.clip(truck_right_vec[2], -1.0, 1.0))
        trailer_roll = math.asin(np.clip(trailer_right_vec[2], -1.0, 1.0))

        # Cámaras (solo en lecturas completas: son lo más costoso y solo sirven para visualizar)
        if completo:
            # Getting camera data (usar el mismo state recién leído para sincronizar proyección)
            reverse_cam_data = self.reverse_cam.poll()
            front_cam_data = self.front_cam.poll()

            # Pasamos el estado del camión o tráiler según corresponda
            z_ruta = self._geometria['z_suelo']
            if is_reverse:
                eje = self._eje_vehiculo(trailer_pos_final, -R_r[:, 0], z_ruta)   # en reversa se avanza hacia atrás
                stream_cam(reverse_cam_data, self.route, estado_trailer, is_reverse=True,
                           pose_camara=self.pose_camara_mundo('reverse', estado_trailer), z_ruta=z_ruta, eje_vehiculo=eje)
            else:
                eje = self._eje_vehiculo(truck_pos_final, R_c[:, 0], z_ruta)
                stream_cam(front_cam_data, self.route, estado_camion, is_reverse=False,
                           pose_camara=self.pose_camara_mundo('front', estado_camion), z_ruta=z_ruta, eje_vehiculo=eje)

        # Return the relevant data for the forward control algorithm
        return {
            "t_sim": self.tiempo_simulacion(),   # tiempo de simulación [s] (ver tiempo_simulacion)
            "v1": v1,
            "psi_truck": psi_truck,
            "psi_trailer": psi_trailer,
            "delta_F2": delta_F2,
            "truck_pos": truck_pos_final,      # en el eje longitudinal del camión
            "truck_dir": dir_truck,
            "truck_roll": truck_roll,
            "trailer_pos": trailer_pos_final,  # en el eje longitudinal del tráiler (desfase medido)
            "trailer_dir": dir_trailer,
            "trailer_roll": trailer_roll
        }
        
        # Yaw del camión - Yaw del trailer = ángulo entre camión y trailer
        # print(f"Yaw Camión: {np.degrees(psi_truck):.2f}°, Yaw Tráiler: {np.degrees(psi_trailer):.2f}°, Articulación: {np.degrees(delta_F2):.2f}°")
        # print(f"Velocidad v1: {v1*(3600/1000):.2f} Km/h")

# Acá generamos el camión y el tráiler
def gen_truck_and_trailer(scenario, bng, direction: str):
    # Obtener todos los vehículos disponibles
    available_vehicles = bng.get_available_vehicles()
    
    # Escogemos la configuración del camión
    truck_configs = available_vehicles['vehicles']['us_semi']['configurations']
    truck_base_cfg = truck_configs['tc82_custom'] 
    truck_cfg_path = f"vehicles/us_semi/{truck_base_cfg['key']}.pc" # La f crea una formatted string, similar a lo que se hace en C

    # Escogemos la configuración del trailer
    trailer_configs = available_vehicles['vehicles']['dryvan']['configurations']
    trailer_base_cfg = trailer_configs['28ft_4500kg'] 
    trailer_cfg_path = f"vehicles/dryvan/{trailer_base_cfg['key']}.pc" # La f crea una formatted string, similar a lo que se hace en C

    # Generamos los camiones y trailers
    truck = Vehicle('truck', model='us_semi', license='Lartrax', part_config=truck_cfg_path)
    trailer = Vehicle('trailer', model='dryvan', license='Lartrax', part_config=trailer_cfg_path)

    orig          = (0, 0, 0.738) # Estas son las coordenadas adecuadas para generar el camión en smallgrid a partir del origen
    route_init     = (20, 0, 0.738) # punto inicial de la ruta generada

    match direction:
        case 'fwd':
            # Parámetros para forward
            trailer_orig  = (-3.017, -0.949, 1.204) # Esta es la ubicación que debe tener el trailer para estar exactamente en el punto donde se puede conectar el camión, esto corresponde a -3.017 a lo largo el eje de movimiento y 0.949 a lo largo del eje de giro
            # Tal diferencia se debe a la programación del origen para trailer y camión
            rot_quat      = (0, 0, 1, -1) # Cuaternion paralelo al eje x # rot_quat = (0, 0, 1, 0) Este cuaternion es paralelo al eje y
        case 'bwd':
            # Parámetros para backward
            trailer_orig  = (3.017, 0.949, 1.204)
            rot_quat      = (0, 0, 1, 1) # cuaternión paralelo a x que mira a las x negativas
        case _:
            raise KeyError("No se recibió 'fwd' ni 'bwd'")

    # Acá escogemos la ruta
    route = make_route(route_init, TIPO_RUTA)   # ver TIPO_RUTA / NOMBRES_RUTA al inicio del archivo


    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=rot_quat)
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=rot_quat)

    return orig, truck, trailer, route
