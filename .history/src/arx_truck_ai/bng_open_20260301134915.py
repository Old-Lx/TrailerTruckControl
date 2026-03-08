from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv # Para leer del .env

def gen_truck_and_trailer(scenario, bng):
    # Obtener todos los vehículos disponibles
    available_vehicles = bng.get_available_vehicles()
    
    # Escogemos la configuración del camión
    truck_configs = available_vehicles['vehicles']['us_semi']['configurations']
    truck_base_cfg = truck_configs['tc82_base'] 
    truck_cfg_path = f"vehicles/us_semi/{truck_base_cfg['key']}.pc" # La f crea una formatted string, similar a lo que se hace en C

    # Escogemos la configuración del trailer
    trailer_configs = available_vehicles['vehicles']['dryvan']['configurations']
    trailer_base_cfg = trailer_configs['28ft'] 
    trailer_cfg_path = f"vehicles/dryvan/{trailer_base_cfg['key']}.pc" # La f crea una formatted string, similar a lo que se hace en C

    # Generamos los camiones y trailers
    truck = Vehicle('truck', model='us_semi', license='Lartrax', part_config=truck_cfg_path)
    trailer = Vehicle('trailer', model='dryvan', license='Lartrax', part_config=trailer_cfg_path)

    orig          = (0, 0, 0) # x = -50, y = -200, z = 0 Para un buen punto en autotest con el camión
    trailer_orig  = (0, -4.9, 0)
    rot_quat      = (0, 0, 1, 0)

    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=rot_quat)
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=rot_quat)

    return orig, truck

def detect_vehicles(bng: BeamNGpy):
    vehicles = bng.vehicles.get_current(False)
    print(vehicles)
    truck = next(v for v in vehicles.values() if v.options['model'] == 'semi')
    truck.connect(bng)        # abre el socket con ese vehículo
    truck.control(throttle=0.5, steering=0.0)

    truck.poll_sensors()          # o scenario.update()
    orig = truck.state['pos']      # (x, y, z)

    return orig, truck

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

    orig, truck = detect_vehicles(bng) # gen_truck_and_trailer(scenario, bng)

    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()

    # Conectar trailer y camión
    print(truck.couplers.attach())


    return bng, truck, orig

    if __name__ == "__main__":
        main()