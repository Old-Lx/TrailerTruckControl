from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_login
from os import getenv
from dotenv import load_dotenv #Para leer del .env

# Para esto se requiere generar un archivo .env y guardar las variables correspondientes
load_dotenv()
bng_home = getenv('BNG_HOME')
bng_user = getenv('BNG_USER')
set_up_simple_login()

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:25252
bng = BeamNGpy('localhost', 25252)#, home=bng_home, user=bng_user)
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model='etk800', license='PYTHON')
# Add it to our scenario at this position and rotation
scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Load and start our scenario
bng.scenario.load(scenario)
bng.scenario.start()
# Make the vehicle's AI span the map
vehicle.ai.set_mode('traffic')
input('Hit Enter when done...')

# Disconnect BeamNG
bng.disconnect()

# Or close the simulator
# bng.close()