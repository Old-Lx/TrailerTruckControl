from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Lidar, State
import cv2
from dotenv import load_dotenv # Para leer del .env
import numpy as np
from os import getenv

def stream_cam(cam_data):
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

            # ¡Acá debo desarrollar algo para generar y detectar las líneas del camino a seguir!

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
            is_visualised=True
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
            is_visualised=True
        )

    # Función para leer los sensores del Truck Trailer
    def read_sensors(self):
        # Leer datos de los sensores
        truck_imu_data = self.imu_truck.poll() # Estos dos son el estado físico del sistema
        trailer_imu_data = self.imu_trailer.poll()
        
        lidar_front_data = self.lidar_front.poll()
        lidar_rear_data = self.lidar_rear.poll()
        reverse_cam_data = self.reverse_cam.poll()
        front_cam_data = self.front_cam.poll()

        # Actualizar el estado físico de los vehículos (para leer el Yaw)
        self.truck.sensors.poll()
        self.trailer.sensors.poll()
        dir_truck = self.truck.state['dir']
        dir_trailer = self.trailer.state['dir']

        # Extraer la velocidad longitudinal (v1) usando el vector de velocidad
        vel_truck = self.truck.state['vel']
        v1 = np.linalg.norm(vel_truck) # Magnitud en m/s

        # Extraer las variables para el modelo cinemático
        # Nota: Verifica en consola cómo viene estructurado imu_data, suele ser una lista de lecturas
        # v1 = imu_data[0]['mass'] ... (dependerá de lo que necesites calcular)
        
        psi_truck = np.arctan2(dir_truck[1], dir_truck[0])
        psi_trailer = np.arctan2(dir_trailer[1], dir_trailer[0])
        delta_F2 = psi_truck - psi_trailer
        
        # Yaw del camión - Yaw del trailer = ángulo entre camión y trailer
        print(f"Yaw Camión: {np.degrees(psi_truck):.2f}°, Yaw Tráiler: {np.degrees(psi_trailer):.2f}°, Articulación: {np.degrees(delta_F2):.2f}°")
        print(f"Velocidad v1: {v1*(3600/1000):.2f} Km/h")


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

    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=rot_quat)
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=rot_quat)

    return orig, truck, trailer

def main():
    # Para esto se requiere generar un archivo .env y guardar las variables correspondientes
    load_dotenv()
    bng_home = getenv('BNG_HOME')
    bng_user = getenv('BNG_USER')

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:25252
    bng = BeamNGpy('localhost', 25252, home=bng_home, user=bng_user)

    # Launch BeamNG.tech
    bng.open()

    # Get the scenarios list
    #scenarios = str(bng.get_levels_and_scenarios())

     # Create a scenario in automation_test_track called Test Land
    scenario = Scenario("smallgrid", "Test Land", description="Implementación de un modelo de control para truck trailer") # tech_ground o autotest son los más despejados

    orig, truck, trailer =  gen_truck_and_trailer(scenario, bng) #  detect_vehicles(bng)

    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()

    truck.connect(bng)
    trailer.connect(bng)

    bng.step(60)

    # Conectar trailer y camión
    truck.couplers.attach()

    bng.step(60)

    truck_trailer = TruckTrailer(truck, trailer, bng) # Convertimos el camión con trailer en un tipo

    truck_trailer.set_sensors()


    return truck_trailer, orig

if __name__ == "__main__":
    main()