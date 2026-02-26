from beamngpy import BeamNGpy
from os import getenv
from dotenv import load_dotenv #Para leer del .env
import ast

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

    return bng

    if __name__ == "__main__":
        main()