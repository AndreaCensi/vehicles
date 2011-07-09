from vehicles import (VehicleSimulation, load_configuration, Configuration,
    instance_world, instance_vehicle)
import numpy as np
from nose.plugins.attrib import attr
import itertools
from vehicles.simulation.vehicle import Vehicle
from vehicles.configuration.instance_all import instance_dynamics
from vehicles.worlds.empty import Empty

# TODO: remove
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
    for k in range(num_episodes): #@UnusedVariable
        sim.new_episode()
        for i in range(num_instants): #@UnusedVariable
            cmds = random_commands(sim.vehicle.commands_spec)
            sim.simulate(cmds, dt)
            sim.compute_observations()
            #print('%3d %3d: %s' % (k, i, list(sensels_values)))

@attr('simulation')
def test_simulation():
    if not Configuration.loaded:
        load_configuration()
        
    worlds = Configuration.worlds
    vehicles = Configuration.vehicles
    for id_world, id_vehicle in itertools.product(worlds, vehicles):
        # XXX: check compatible
        # if compatible
        vehicle = instance_vehicle(id_vehicle)
        world = instance_world(id_world)
        simulation = VehicleSimulation(vehicle, world)
        num_episodes = 3
        num_instants = 100
        dt = 0.1
        yield check_simulation, simulation, num_episodes, num_instants, dt

def check_simulation_one_step(sim, dt):
    ''' Tries to run one step of the simulation. '''
    sim.new_episode()    
    cmds = random_commands(sim.vehicle.commands_spec)
    sim.simulate(cmds, dt)
    sim.compute_observations()

from .. import logger

def test_simulation_one_step():
    ''' Tries one step of simulation. '''
    if not Configuration.loaded:
        load_configuration()
        
    worlds = Configuration.worlds
    vehicles = Configuration.vehicles
    for id_world, id_vehicle in itertools.product(worlds, vehicles):
        yield check_simulation_one_step_conf, id_world, id_vehicle

def check_simulation_one_step_conf(id_world, id_vehicle):
    check_simulation_one_step_conf.description = \
        'One step %r %r' % (id_world, id_vehicle)
    vehicle = instance_vehicle(id_vehicle)
    world = instance_world(id_world)
    sim = VehicleSimulation(vehicle, world)
    dt = 1
    try: 
        sim.new_episode()
        cmds = random_commands(sim.vehicle.commands_spec)
        sim.simulate(cmds, dt)
        sim.compute_observations()
    except:
        logger.error('Error for vehicle=%s world=%s' % (id_vehicle, id_world))
        raise
        
def test_with_blind_robots():
    ''' 
        In this test, all dynamics are instantiated and connected to a blind
        vehicle.
    '''
    for id_dynamics in Configuration.dynamics:
        yield check_blind_robot, id_dynamics    
        
def check_blind_robot(id_dynamics):
    check_blind_robot.__dict__['description'] = 'check_blind_robot(%r)' % id_dynamics
    vehicle = Vehicle()
    dynamics = instance_dynamics(id_dynamics)
    vehicle.add_dynamics(id_dynamics, dynamics)
    world = Empty([[-10, 10], [-10, 10], [-10, 10]])
    sim = VehicleSimulation(vehicle, world)
    check_simulation(sim, num_episodes=3, num_instants=10, dt=0.1)

    
    
