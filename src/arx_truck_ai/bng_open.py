from beamngpy import BeamNGpy, Scenario
from dotenv import load_dotenv # Para leer del .env
import numpy as np
from os import getenv
from arx_truck_ai.truck_trailer import TruckTrailer, gen_truck_and_trailer

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

    direction = 'bwd'

    orig, truck, trailer, route =  gen_truck_and_trailer(scenario, bng, direction) #  detect_vehicles(bng)

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
    truck_trailer.truck_origin = orig # el front cam origin
    truck_trailer.route = route

    truck_trailer.set_sensors()


    return truck_trailer, orig

if __name__ == "__main__":
    main()