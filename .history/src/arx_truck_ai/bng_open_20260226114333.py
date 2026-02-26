from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv #Para leer del .env

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
    # Get available vehicles
    available_vehicles = bng.get_available_vehicles()
    scenarios = bng.get_levels_and_scenarios()

    file_path1 = 'vehicles.txt'
    file_path2 = 'scenarios.txt'

    with open(file_path1, 'w') as file:
        file.write(available_vehicles)
    with open(file_path2, 'w') as file:
        file.write(scenarios)

     # Create a scenario in west_coast_usa called Test Land
    scenario = Scenario("west_coast_usa", "Test Land", description="Implementación de un modelo de control para truck trailer")
    # Create an ETK800 with the licence plate 'PYTHON'
    vehicle = Vehicle('truck', model='us_semi', license='Lartrax')

    return bng, scenario, vehicle

    if __name__ == "__main__":
        main()