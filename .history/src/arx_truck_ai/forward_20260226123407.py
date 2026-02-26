import numpy as np
from arx_truck_ai import bng_open

def main():
    bng, scenario, vehicle, orig = bng_open.main()

    # Posición de origen para el vehículo

    bng.control.pause()
    bng.settings.set_deterministic(60)

    script = []

    points = []
    point_color = (0, 0, 0, 0.1)
    sphere_coordinates = []
    sphere_radii = []
    sphere_colors = []

    # Ruta senoidal
    for i in range(5):
        node = {
            #  Calculate the position as a sine curve that makes the vehicle
            #  drive from left to right. The z-coordinate is not calculated in
            #  any way because `ai.set_script` by default makes the polyline to
            #  follow cling to the ground, meaning the z-coordinate will be
            #  filled in automatically.
            "x": 4 * np.sin(np.radians(i)) + orig[0],
            "y": i * 0.2 + orig[1],
            "z": orig[2],
            #  Calculate timestamps for each node such that the speed between
            #  points has a sinusoidal variance to it.
            "t": (2 * i + (np.abs(np.sin(np.radians(i)))) * 64) / 64,
        }
        script.append(node)
        points.append((node["x"], node["y"], node["z"]))

        if i % 10 == 0:
            sphere_coordinates.append((node["x"], node["y"], node["z"]))
            sphere_radii.append(np.abs(np.sin(np.radians(i))) * 0.25)
            sphere_colors.append((np.sin(np.radians(i)), 0, 0, 0.8))

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()

    
    vehicle.set_velocity(-7)

    bng.debug.add_spheres(
        sphere_coordinates, sphere_radii, sphere_colors, cling=True, offset=0.1
    )
    bng.debug.add_polyline(points, point_color, cling=True, offset=0.1)

    # bng.traffic.spawn()

    # Make the vehicle's AI span the map
    vehicle.ai.set_script(script)

    for i in range(65):
        bng.control.step(60)

    bng.control.resume()

    input('Presione enter cuando cargue la simulación...')

    # Disconnect BeamNG
    bng.disconnect()

    # Or close the simulator
    # bng.close()

    if __name__ == "__main__":
        main()