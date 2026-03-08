from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv # Para leer del .env

def is_coupled(bng, truck_vid, trailer_vid=None):
    info = bng.get_current_vehicles_info(include_config=False)
    truck_info = info.get(truck_vid, {})
    print("Couplers:", truck_info.get("couplers"))
    if not truck_info:
        return False

    for coupler in truck_info.get('couplers', {}).values():
        if coupler.get('state') != 'coupled':
            continue
        if trailer_vid and coupler.get('otherVehicle') != trailer_vid:
            continue
        return True
    return False

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

    # Get the scenarios list
    #scenarios = str(bng.get_levels_and_scenarios())

     # Create a scenario in automation_test_track called Test Land
    scenario = Scenario("autotest", "Test Land", description="Implementación de un modelo de control para truck trailer") # tech_ground o autotest son los más despejados
    
    # Generamos los camiones y trailers
    truck = Vehicle('truck', model='us_semi', license='Lartrax', part_config=truck_cfg_path)
    trailer = Vehicle('trailer', model='dryvan', license='Lartrax', part_config=trailer_cfg_path)

    orig          = (-50.0, -200.0, 0.5) # x = -50, y = -200, z = 0 Para un buen punto en autotest con el camión
    trailer_orig  = (-49.0, -205.5, 0.5)  # 5.5 metros detrás
    rot_quat      = (0, 0, 1, 0)

    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=rot_quat)
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=rot_quat)
    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()

    # Conectar trailer y camión
    while True:
        truck.control(gear=-1, throttle=0.15)
        bng.step(0.016)
        exit = is_coupled(bng, truck.vid, trailer.vid)

        if (exit):
            truck.control(throttle=0, brake=1)  # se detiene cuando ya está acoplado
            break


    return bng, truck, orig

    if __name__ == "__main__":
        main()