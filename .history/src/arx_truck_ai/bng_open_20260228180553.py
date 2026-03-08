from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv # Para leer del .env

def main():
    # Para esto se requiere generar un archivo .env y guardar las variables correspondientes
    load_dotenv()
    bng_home = getenv('BNG_HOME')
    bng_user = getenv('BNG_USER')
    pc_route = getenv('PART_CONFIG_ROUTE')

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:25252
    bng = BeamNGpy('localhost', 25252, home=bng_home, user=bng_user)

    # Launch BeamNG.tech
    bng.open()
    
    # Obtener todos los vehículos disponibles
    available_vehicles = bng.get_available_vehicles()
    truck_entry = available_vehicles['vehicles']['us_semi']
    truck_cfg = truck_entry['configurations']['tc82_base']
    print(truck_cfg['key'])

    # Get the scenarios list
    #scenarios = str(bng.get_levels_and_scenarios())

     # Create a scenario in automation_test_track called Test Land
    scenario = Scenario("tech_ground", "Test Land", description="Implementación de un modelo de control para truck trailer")
    
    # Generamos los camiones y trailers
    truck = Vehicle('truck', model='us_semi', license='Lartrax', part_config=pc_route + truck_cfg['key'] + ".PC")
    trailer = Vehicle('trailer', model='containerTrailer', license='Lartrax')

    orig = (-50, -400, 0.5) # x = -50, y = -200, z = 0 Para un buen punto en autotest con el camión
    trailer_orig = (-50, -406,3, 1)

    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(truck, pos=orig, rot_quat=(0, 0, 1, 0))
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=(0, 0, 1, 0))
    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()

    # Camion y trailer ya creados y en la escena
    truck.couplers.attach()         # acopla todos los couplers disponibles

    return bng, truck, orig

    if __name__ == "__main__":
        main()