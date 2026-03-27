from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Lidar, State
import cv2
import numpy as np
import time
import math

# Construye la matriz intrínseca K de la cámara para usar en cv2.projectPoints.
def get_camera_intrinsics(fov_y_deg: float, width: int, height: int) -> np.ndarray:
    fov_y_rad = math.radians(fov_y_deg)
    f_y = (height / 2.0) / math.tan(fov_y_rad / 2.0)
    f_x = f_y  # BeamNG usa píxeles cuadrados

    c_x = width / 2.0
    c_y = height / 2.0

    K = np.array([
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0, 0, 1]
    ], dtype=np.float32)
    return K

"""Transformación homogénea cámara-en-vehículo.

    Usa los ejes locales de la cámara definidos en el propio sensor (`dir`, `up`).
    En BeamNG, `dir` apunta al frente de la cámara y `up` al techo del vehículo.
    """
def _build_T_vc(cam_pos: tuple, cam_dir: tuple, cam_up: tuple) -> np.ndarray:

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

"""Transformación homogénea vehículo-en-mundo usando `state` de BeamNG.

    Tomamos la convención más simple: ejes locales del camión son
    +X derecha, +Y frente, +Z arriba. `state['dir']` apunta al frente real del
    vehículo en coordenadas globales, `state['up']` al techo; reconstruimos el eje
    derecho con un cruzado. R_wv queda con columnas (right, forward, up).
    """
def _build_T_wv(truck_state: dict) -> np.ndarray:

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

"""Devuelve (rvec, tvec) usando la convención real de BeamNG para el vehículo.

    Convención local del vehículo en BeamNG: +X = izquierda, +Y = atrás, +Z = arriba.
    Por tanto, el frente (hacia donde mira) es -Y local. Reconstruimos la rotación
    vehículo→mundo con columnas (-r, -f, u) para que cuando el camión mira a -Y,
    R_veh sea identidad. Luego proyectamos a OpenCV (eje y hacia abajo).
    """
def get_camera_extrinsics(truck_state: dict, cam_pos: tuple, cam_dir: tuple, cam_up: tuple):

    truck_p = np.array(truck_state['pos'], dtype=np.float32)
    truck_f = np.array(truck_state['dir'], dtype=np.float32)  # Frente global (-Y local)
    truck_u = np.array(truck_state['up'], dtype=np.float32)   # Techo global (+Z local)

    truck_f /= np.linalg.norm(truck_f)
    truck_u /= np.linalg.norm(truck_u)

    # Gram-Schmidt para ortonormalizar y eliminar deriva que causa desplazamiento en Y
    truck_r = np.cross(truck_f, truck_u)
    truck_r /= np.linalg.norm(truck_r)
    truck_f = np.cross(truck_u, truck_r)
    truck_f /= np.linalg.norm(truck_f)
    truck_u = np.cross(truck_r, truck_f)
    truck_u /= np.linalg.norm(truck_u)

    # Rotación local→mundo del vehículo
    # Local BeamNG (con Camera): +X = derecha, +Y = frente (mira hacia -Y global), +Z = arriba.
    # Por eso usamos columnas (right, -front, up). Ortonormal y sin sesgo lateral.
    R_veh = np.column_stack((truck_r, -truck_f, truck_u))

    # Posición y orientación de la cámara en mundo (BeamNG)
    cam_pos_world = truck_p + R_veh.dot(np.array(cam_pos, dtype=np.float32))
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

def gen_x_line(origin: tuple[float, float, float]) -> list:
    route = []

    for i in range(2400):
        node = [
            (0.1 * i) + origin[0], # Debo revisar qué aumentos de x convienen
            origin[1],
            0.01,
        ]
        route.append(node)

    return route

def gen_x_sine(origin: tuple[float, float, float]) -> list:
    route = []

    for i in range(2400):
        node = [
            4 * np.sin(np.radians(i)) + origin[0],
            i * 0.2 + origin[1],
            0.01, # Hay que verificar cómo calcular z para terrenos no planos
        ]
        route.append(node)

    return route

'''
Acá creamos los puntos de la ruta que seguirá el sistema truck-trailer
    route_type:
    0: Línea recta sobre el eje X
    1: Línea recta sobre el eje Y
    2: Línea recta diagonal en el plano XY
    3: Trayectoria parabólica y = x^2
    4: Trayectoria parabólica x = y^2
    5: Trayectoria senosoidal en x
    6: Trayectoria senosoidal en y

'''
def make_route(origin: tuple[float, float, float], route_type: int) -> list: 
    match route_type:
        case 0:
            return  gen_x_line(origin)
        case 5:
            return gen_x_sine(origin)
        case _:
            return

# Proyecta la ruta (coords mundo) a la imagen de la front_cam. Se apoya en la pose del vehículo (`state`) + el offset fijo de la cámara para mantener la polilínea anclada al mundo aunque el camión se mueva.
def stream_cam(cam_data: dict, route: list, truck_state: dict):

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

        # Parámetros EXACTOS de la configuración de tu front_cam
        cam_local_pos = (0, -0.216, 2.784)
        cam_local_dir = (0, -0.965, -0.259)
        cam_local_up = (0, 0, 1)

        # Extrínseca = pose vehículo (state) + offset fijo de la cámara
        rvec, tvec = get_camera_extrinsics(truck_state, cam_local_pos, cam_local_dir, cam_local_up)

        # Intrínseca de la cámara de BeamNG (70° y 512x512)
        K = get_camera_intrinsics(fov_y_deg=70, width=512, height=512)

        route_np = np.array(route, dtype=np.float32)

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

        # Visualizar el streaming en una ventana emergente
        cv2.imshow("Video en Streaming - Route Cam", img_bgr)

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

    # Función para asignar los sensores
    def set_sensors(self):
        # Sensores del camión

        # Estado del camión
        self.state_truck = State()
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
            pos=(0, -0.216, 2.784),     # En la parte del techo del camión en el frente
            dir=(0, -0.965, -0.259),  # Viendo 15 grados hacia abajo del eje Y en el plano YZ
            up=(0, 0, 1),
            resolution=(512, 512),
            field_of_view_y= 70, # 70 es el default
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
            pos=(0, 9.148, 3.960),      # En la parte de atrás y arriba del trailer
            dir=(0, 1, 0), 
            up=(0, 0, 1),
            resolution=(512, 512),
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
    def read_sensors(self):
        # Leer datos de los sensores
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
        delta_F2 = psi_truck - psi_trailer

        # Getting camera data (usar el mismo state recién leído para sincronizar proyección)
        reverse_cam_data = self.reverse_cam.poll()
        front_cam_data = self.front_cam.poll()

        init_time = time.perf_counter()
        # Pasamos el estado del camión
        stream_cam(front_cam_data, self.route, estado_camion)
        end_time = time.perf_counter()
        print(f"Test en {(end_time - init_time):.4f}")

        # Yaw del camión - Yaw del trailer = ángulo entre camión y trailer
        # print(f"Yaw Camión: {np.degrees(psi_truck):.2f}°, Yaw Tráiler: {np.degrees(psi_trailer):.2f}°, Articulación: {np.degrees(delta_F2):.2f}°")
        # print(f"Velocidad v1: {v1*(3600/1000):.2f} Km/h")

# Acá generamos el camión y el tráiler
def gen_truck_and_trailer(scenario, bng):
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

    orig          = (0, 0, 0.738) #  Estas son las coordenadas adecuadas para generar el camin en smallgrid
    trailer_orig  = (-3.017, -0.949, 1.204) # Esta es la ubicación que debe tener el trailer para estar exactamente en el punto donde se puede conectar el camión, esto corresponde a -3.017 a lo largo el eje de movimiento y 0.949 a lo largo del eje de giro
    # Tal diferencia se debe a la programación del origen para trailer y camión
    rot_quat      = (0, 0, 1, -1) # Cuaternion paralelo al eje x # rot_quat = (0, 0, 1, 0) Este cuaternion es paralelo al eje y

    route = make_route(orig, 0)


    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=rot_quat)
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=rot_quat)

    return orig, truck, trailer, route
