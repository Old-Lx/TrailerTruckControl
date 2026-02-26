from beamngpy import BeamNGpy, Scenario, Vehicle
from os import getenv
from dotenv import load_dotenv #Para leer del .env

# Para esto se requiere generar un archivo .env y guardar las variables correspondientes
load_dotenv()
bng_home = getenv('BNG_HOME')
bng_user = getenv('BNG_USER')

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:25252
bng = BeamNGpy('localhost', 25252, home=bng_home, user=bng_user)
print(bng_home)
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'ai_sine'
scenario = Scenario("west_coast_usa", "ai_sine")
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model='etk800', license='Lartrax')
# Posición de origen para el vehículo
orig = (-769.1, 400.8, 142.8)
# Add it to our scenario at this position and rotation
scenario.add_vehicle(vehicle, pos=orig, rot_quat=(0, 0, 0.3826834, 0.9238795))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Load and start our scenario
bng.scenario.load(scenario)
bng.scenario.start()

# beamng.traffic.spawn()

# Make the vehicle's AI span the map
vehicle.ai.set_mode('traffic')
input('Hit Enter when done...')

# Disconnect BeamNG
bng.disconnect()

# Or close the simulator
# bng.close()