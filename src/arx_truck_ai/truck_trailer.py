from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Lidar
import cv2
import numpy as np
import time

def gen_x_line(origin: tuple[float, float, float]) -> list:
    route = []

    for i in range(2400):
        node = [
            (0.1 * i) + origin[0], # Debo revisar qué aumentos de x convienen
            origin[1],
            origin[2],
        ]
        route.append(node)

    return route

def gen_x_sine(origin: tuple[float, float, float]) -> list:
    route = []

    for i in range(2400):
        node = [
            4 * np.sin(np.radians(i)) + origin[0],
            i * 0.2 + origin[1],
            origin[2], # Hay que verificar cómo calcular z para terrenos no planos
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

# Rutina para mostrar la imagen de la cámara
def stream_cam(cam_data: dict, cam: Camera, route: list):
    # Extraer la imagen a color
        if 'colour' in cam_data:
            # BeamNGpy suele entregar una imagen de formato PIL RGBA
            img_pil = cam_data['colour']
            
            # Convertir de PIL Image a un Array de Numpy (para OpenCV)
            img_array = np.array(img_pil)

            # Convertir el formato de color de RGBA (BeamNG) a BGR (El estándar de OpenCV)
            if img_array.shape[2] == 4: # Si tiene canal Alpha (transparencia)
                img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
            else:
                img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

            # https://www.geeksforgeeks.org/computer-vision/mapping-coordinates-from-3d-to-2d-using-opencv-python/
            # No uso este método porque BeamNG tiene una función más sencilla que logra obtener los pixeles
            # cv2.projectPoints()
            # Dibujo la ruta
            puntos_en_pantalla = []
        
            for punto_3d in route:
                # Pixeles a marcar
                pixel = cam.world_point_to_pixel(punto_3d)
                
                # BeamNG devuelve None o coordenadas negativas si el punto no es visible o hay un error matemático
                if pixel is not None:
                    x, y = int(pixel[0]), int(pixel[1])
                    
                    # Validar que el píxel caiga dentro de la resolución de tu cámara (512x512)
                    if 0 <= x < 512 and 0 <= y < 512:
                        puntos_en_pantalla.append([x, y])
            
            # Si capturamos suficientes puntos visibles, trazamos la línea
            if len(puntos_en_pantalla) > 1:
                # OpenCV requiere que los puntos tengan una forma matricial específica (-1, 1, 2)
                puntos_np = np.array(puntos_en_pantalla, np.int32).reshape((-1, 1, 2))
                
                # Dibujamos una línea verde (0, 255, 0) de grosor 3
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
            is_streaming=True
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
            is_streaming=True
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
        dir_truck = self.truck.state['dir']
        dir_trailer = self.trailer.state['dir']

        # Extraer la velocidad longitudinal (v1) usando el vector de velocidad
        vel_truck = self.truck.state['vel']
        v1 = np.linalg.norm(vel_truck) # Magnitud en m/s

        # Extraer las variables para el modelo cinemático
        # v1 = imu_data[0]['mass']
        
        psi_truck = np.arctan2(dir_truck[1], dir_truck[0])
        psi_trailer = np.arctan2(dir_trailer[1], dir_trailer[0])
        delta_F2 = psi_truck - psi_trailer

        # Getting camera data
        reverse_cam_data = self.reverse_cam.poll()
        front_cam_data = self.front_cam.poll()

        init_time = time.perf_counter()
        stream_cam(front_cam_data, self.front_cam, self.route)
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
