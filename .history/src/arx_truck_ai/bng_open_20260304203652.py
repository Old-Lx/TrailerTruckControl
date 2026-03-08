from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Lidar, State
from dotenv import load_dotenv # Para leer del .env
import numpy as np
from os import getenv
from time import sleep

# Clase Camión con Trailer
class TruckTrailer:
    # Sensores
    imu_truck: AdvancedIMU
    imu_trailer: AdvancedIMU
    lidar_front: Lidar
    lidar_rear: Lidar
    line_cam: Camera
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
            "accel1", self.bng, self.truck,
            # From Lua: args.pos / dir / up
            pos=(0, 0, 0.8),  # placing the IMU on vehicle roof (with snapping)
            dir=(0, 0, 0),
            up=(-0, 0, 1),

            # Update intervals - set to 2000Hz
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
        # IMU para medir posición y orientación del trailer
        self.imu_trailer = AdvancedIMU(
            "imu_trailer", self.bng, self.trailer,
            pos=(0, 0, 2.2), dir=(0, 0, 0), up=(0, 0, 1), # Ajustado al techo del tráiler
            gfx_update_time=0.0005, physics_update_time=0.0005,
            smoother_strength=1.0, is_using_gravity=True, 
            is_visualised=True
        )
        # Cámara para seguir la ruta
        self.line_cam = Camera(
            'line_cam', self.bng, vehicle=self.trailer, 
            pos=(0, 0, 2.0), dir=(0, 0, -1), resolution=(320, 180)
        )

        # Sensores del trailer
        # No se simula el sensor del ángulo entre camión y trailer porque no hace falta

        # Cámara de marcha hacia atrás
        self.reverse_cam = Camera(
            'reverse_cam', self.bng, vehicle=self.trailer,
            pos=(0.0, 0.0, 2.2), dir=(0.0, 0.0, -1.0), resolution=(640, 360)
        )
        # Sensor para los golpes marcha hacia delante
        self.lidar_front = Lidar(
            "lidar_front", self.bng, self.truck,
            requested_update_time=0.01, is_using_shared_memory=True,
            is_360_mode=False,           # ¡Modo direccional activado!
            horizontal_angle=180,        # Cubre 180° hacia el frente
            pos=(0.0, -4.5, 1.5),        # Aprox. en el parachoques/capó del us_semi
            dir=(0.0, -1.0, 0.0),        # Apuntando hacia adelante (Eje -Y en BeamNG)
            vertical_resolution=16, frequency=20, is_visualised=True
        )
        # Sensor para los golpes marcha hacia atrás
        self.lidar_rear = Lidar(
            "lidar_rear", self.bng, self.trailer,
            requested_update_time=0.01, is_using_shared_memory=True,
            is_360_mode=False,           # ¡Modo direccional activado!
            horizontal_angle=180,        # Cubre 180° hacia atrás
            pos=(0.0, 4.5, 3.5),         # Aprox. borde superior trasero del dryvan
            dir=(0.0, 1.0, 0.0),         # Apuntando hacia atrás (Eje +Y en BeamNG)
            vertical_resolution=16, frequency=20, is_visualised=True
        )

    # Función para leer los sensores del Truck Trailer
    def read_sensors(self):
        print("Hola")
        # Leer datos de los sensores
        truck_imu_data = self.imu_truck.poll()
        trailer_imu_data = self.imu_trailer.poll()
        
        lidar_front_data = self.lidar_front.poll()
        lidar_rear_data = self.lidar_rear.poll()
        lidar_data = self.lidar.poll()
        reverse_cam_data = self.reverse_cam.poll()
        line_cam_data = self.line_cam.poll()

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

        # Procesar la nube de puntos del lidar
        guard_points = np.array(lidar_data['pointCloud']).reshape(-1, 3)
        if guard_points.size > 0:
            min_radius = np.min(np.linalg.norm(guard_points[:, :2], axis=1))
            # collision_safe = min_radius > max(3.0, 0.5 * v1)
            
        print(f"Yaw Camión: {np.degrees(psi_truck):.2f}°, Yaw Tráiler: {np.degrees(psi_trailer):.2f}°, Articulación: {np.degrees(delta_F2):.2f}°")
        print(f"Velocidad v1: {v1:.2f} m/s")

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

    truck_trailer = TruckTrailer(truck, trailer, bng)

    truck_trailer.set_sensors()


    return truck_trailer, orig

if __name__ == "__main__":
    main()