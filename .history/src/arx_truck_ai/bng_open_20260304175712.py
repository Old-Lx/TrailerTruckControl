from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, IMUSensor, LidarSensor
from os import getenv
from dotenv import load_dotenv # Para leer del .env

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
    trailer_orig  = (0.949, -3.017, 1.204) # Esta es la ubicación que debe tener el trailer para estar exactamente en el punto donde se puede conectar el camión, esto corresponde a -3.017 a lo largo el eje de movimiento y 0.949 a lo largo del eje de giro
    # Tal diferencia se debe a la programación del origen para trailer y camión
    rot_quat      = (0, 0, 1, 0)

    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=rot_quat)
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=rot_quat)

    # Sensores del camión
    truck.attach_sensor('imu_main', IMUSensor(pos=(0,0,0.8), dir=(0,0,0)))
    trailer.attach_sensor('line_cam', Camera(pos=(0, 0, 2.0), dir=(0, 0, -1), fov=70, resolution=(320, 180), colour=True, is_render_colours=True))

    # Sensores del trailer
    # No se simula el sensor del ángulo entre camión y trailer porque no hace falta
    trailer.attach_sensor(
    'lidar_guard',
    LidarSensor(
        pos=(-1.0, 0.0, 2.0),   # sobre la quinta rueda
        dir=(0.0, 0.0, 0.0),
        vertical_resolution=16,
        horizontal_resolution=720,  # 0.5° por rayo
        vertical_fov=30,
        max_distance=80,
        update_freq=20,
        is_visualised=False
    )
)

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


    return bng, truck, orig

    if __name__ == "__main__":
        main()