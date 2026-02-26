from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv #Para leer del .env
import ast

# Turns a dictionary into a class // extraído de https://www.geeksforgeeks.org/python/how-to-change-a-dictionary-into-a-class/
class Dict2Class(object):
    
    def __init__(self, my_dict):
        
        for key in my_dict:
            setattr(self, key, my_dict[key])

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
    available_vehicles = ast.literal_eval(bng.get_available_vehicles())
    scenarios = ast.literal_eval(bng.get_level_scenarios())

     # Create a scenario in west_coast_usa called Test Land
    scenario = Scenario("west_coast_usa", "Test Land", description="Implementación de un modelo de control para truck trailer")
    # Create an ETK800 with the licence plate 'PYTHON'
    vehicle = Vehicle('truck', model='us_semi', license='Lartrax')

    return bng, scenario, vehicle

    if __name__ == "__main__":
        main()