import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from Simulation import Simulation

def main():
    sim = Simulation()

#     client = RemoteAPIClient()
#     sim = client.getObject('sim')
#     try:
#         # Start simulation
#         sim.startSimulation()

#         # Get the quadcopter target
#         target = sim.getObject("/target")

#         waypoints = [
#             [62.925, 16.7, 1.0],   # Takeoff to 1m height
#         ]

#         for point in waypoints:
#             set_target(sim, target, point[0], point[1], point[2])
#             time.sleep(4)  # Wait 4 seconds for drone to fly to the waypoint

#         # Stop simulation
#         sim.stopSimulation()
#     except(Exception):
#         sim.stopSimulation

# def set_target(sim, target, x, y, z):
#     sim.setObjectPosition(target, -1, [x, y, z])
#     print(f"Moving quadcopter to target: ({x}, {y}, {z})")

main()