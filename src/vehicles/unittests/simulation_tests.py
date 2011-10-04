from .. import VehicleSimulation, VehiclesConfig, logger, Vehicle
from ..worlds import Empty
from nose.plugins.attrib import attr
import itertools
import numpy as np

def random_commands(commands_spec):
    values = []
    for i, kind in enumerate(commands_spec['format']):
        lower, upper = commands_spec['range'][i]
        if kind == 'C':
            value = lower + np.random.rand() * (upper - lower)
        elif kind == 'D':
            value = np.random.randint(lower, upper)
        else: 
            raise ValueError('Invalid kind %r in %s.' % (kind, commands_spec))
        values.append(value)
    return values

def check_simulation(sim, num_episodes, num_instants, dt):
    for k in range(num_episodes): #@UnusedVariable
        sim.new_episode()
        for i in range(num_instants): #@UnusedVariable
            cmds = random_commands(sim.vehicle.dynamics.get_commands_spec())
            sim.simulate(cmds, dt)
            sim.compute_observations()

@attr('simulation')
def test_simulation():
    VehiclesConfig.load()
        
    worlds = VehiclesConfig.worlds
    vehicles = VehiclesConfig.vehicles
    for id_world, id_vehicle in itertools.product(worlds, vehicles):
        # XXX: check compatible
        # if compatible
        vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
        world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
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


def test_simulation_one_step():
    ''' Tries one step of simulation. '''
    VehiclesConfig.load()
        
    worlds = VehiclesConfig.worlds
    vehicles = VehiclesConfig.vehicles
    for id_world, id_vehicle in itertools.product(worlds, vehicles):
        yield check_simulation_one_step_conf, id_world, id_vehicle

def check_simulation_one_step_conf(id_world, id_vehicle):
    check_simulation_one_step_conf.description = \
        'One step %r %r' % (id_world, id_vehicle)
    vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
    world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
    sim = VehicleSimulation(vehicle, world)
    dt = 1
    try: 
        sim.new_episode()
        cmds = random_commands(sim.vehicle.dynamics.get_commands_spec())
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
    for id_dynamics in VehiclesConfig.dynamics:
        yield check_blind_robot, id_dynamics    
        
def check_blind_robot(id_dynamics):
    desc = 'check_blind_robot(%r)' % id_dynamics
    check_blind_robot.__dict__['description'] = desc 
    vehicle = Vehicle()
    dynamics = VehiclesConfig.dynamics.instance(id_dynamics) #@UndefinedVariable
    vehicle.add_dynamics(id_dynamics, dynamics)
    world = Empty([[-10, 10], [-10, 10], [-10, 10]])
    sim = VehicleSimulation(vehicle, world)
    check_simulation(sim, num_episodes=3, num_instants=10, dt=0.1) 
    
