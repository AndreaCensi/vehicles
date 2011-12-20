from vehicles.simulation.vehicle import Vehicle
from vehicles.simulation.simulation import VehicleSimulation
from vehicles.unittests.fps_test import check_simulation
from vehicles.worlds.empty_world import Empty


def check_blind_robot(id_dynamics, dynamics):
    vehicle = Vehicle()
    vehicle.add_dynamics(id_dynamics, dynamics)
    world = Empty([[-10, 10], [-10, 10], [-10, 10]])
    sim = VehicleSimulation(vehicle, world)
    check_simulation(sim, num_episodes=3, num_instants=10, dt=0.1)

