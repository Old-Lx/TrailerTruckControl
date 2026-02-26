from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv # Para leer del .env

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
    #available_vehicles = str(bng.get_available_vehicles())
    # Get the scenarios list
    #scenarios = str(bng.get_levels_and_scenarios())

     # Create a scenario in automation_test_track called Test Land
    scenario = Scenario("autotest", "Test Land", description="Implementación de un modelo de control para truck trailer")
    # Create an ETK800 with the licence plate 'PYTHON'
    vehicle = Vehicle('truck', model='us_semi', license='Lartrax')
    trailer = Vehicle('trailer', model='containerTrailer', license='Lartrax')

    orig = (-50, -400, -1) # x = -50, y = -200, z = 0 Para un buen punto en autotest con el camión
    trailer_orig = (-50, -408, 0)

    # Add it to our scenario at this position and rotation
    scenario.add_vehicle(vehicle, pos=orig, rot_quat=(0, 0, 1, 0))
    scenario.add_vehicle(trailer, pos=trailer_orig, rot_quat=(0, 0, 1, 0))

    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    return bng, scenario, vehicle, orig

    if __name__ == "__main__":
        main()