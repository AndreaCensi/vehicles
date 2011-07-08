
from ..loading import load_configuration, Configuration
from simple_vehicles.loading.instance_all import instance_all, instance_vehicle
from simple_vehicles.olympics.vehicle_simulation import VehicleSimulation
import numpy as np

def random_command(command_spec):
    if isinstance(command_spec, tuple):
        lower, upper = command_spec
        return lower + np.random.rand() * (upper - lower)  
    elif isinstance(command_spec, list):
        n = len(command_spec)
        return command_spec[np.random.randint(n)]
    else:
        raise ValueError()
        
def random_commands(commands_spec):
    return [random_command(x) for x in commands_spec]

def check_simulation(sim, num_episodes, num_instants, dt):
    for k in range(num_episodes):
        sim.new_episode()
        for i in range(num_instants):
            cmds = random_commands(sim.commands_spec)
            sim.simulate(cmds, dt)
            sensels_values = sim.compute_observations()
            #print('%3d %3d: %s' % (k, i, list(sensels_values)))

def test_simulation():
    if not Configuration.loaded:
        load_configuration()
        
    for id_world in Configuration.worlds:
         
        for id_vehicle in Configuration.vehicles:
            # if compatible
            simulation = VehicleSimulation(id_vehicle, id_world)
            num_episodes = 3
            num_instants = 100
            dt = 0.1
            yield check_simulation, simulation, num_episodes, num_instants, dt
