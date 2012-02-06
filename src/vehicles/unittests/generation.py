from . import (all_dynamics, get_dynamics, logger, get_sensor, all_sensors,
    all_worlds, get_vehicle, all_vehicles, get_world)
from nose.tools import istest
import sys


def add_to_module(function, module_name):
    module = sys.modules[module_name]
    name = function.__name__

    if not 'test' in name:
        raise Exception('No "test" in function name %r' % name)

    if not 'test' in module_name:
        raise Exception('While adding %r in %r: module does not have'
                        '"test" in it, so nose will not find the test.' %
                        (name, module_name))

    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)

    module.__dict__[name] = function


#
#def add_to_module(function, module_name):
#    module = sys.modules[module_name]
#    if not 'test' in module_name:
#        logger.error('Warning: Nose will not find tests in %r.' % module_name)
#    
#
#    
#    if name in module.__dict__:
#        raise Exception('Already created test %r.' % name)
#    module.__dict__[name] = function
#
#    if not 'test' in module_name:
#        raise Exception('While adding %r in %r: module does not have "test"'
#                        ' in it, so nose will not find the test.' %
#                        (name, module_name))
#
#    #print('Add %s: %s' % (module_name, name))


def add_dynamics_f(f, id_dynamics):
    @istest
    def test_caller():
        dynamics = get_dynamics(id_dynamics)
        wrap_with_desc(f, (id_dynamics, dynamics), dynamics=dynamics)
    test_caller.dynamics = id_dynamics
    test_caller.__name__ = 'test_%s_%s' % (f.__name__, id_dynamics)
    add_to_module(test_caller, f.__module__)


def add_pair_f(f, id_dynamics, id_sensor):
    @istest
    def test_caller():
        dynamics = get_dynamics(id_dynamics)
        sensor = get_sensor(id_sensor)
        wrap_with_desc(f, (id_dynamics, dynamics, id_sensor, sensor),
                       dynamics=dynamics, sensor=sensor)
    test_caller.__name__ = 'test_%s_%s_%s' % (f.__name__,
                                              id_dynamics, id_sensor)
    test_caller.dynamics = id_dynamics
    test_caller.sensor = id_sensor
    add_to_module(test_caller, f.__module__)


def add_world_vehicle_f(f, id_world, id_vehicle):
    @istest
    def test_caller():
        world = get_world(id_world)
        vehicle = get_vehicle(id_vehicle)
        wrap_with_desc(f, (id_world, world, id_vehicle, vehicle),
                            world=world, vehicle=vehicle)
    test_caller.__name__ = 'test_%s_%s_%s' % (f.__name__, id_world, id_vehicle)
    test_caller.world = id_world
    test_caller.vehicle = id_vehicle
    add_to_module(test_caller, f.__module__)


def add_world_f(f, id_world):
    @istest
    def test_caller():
        world = get_world(id_world)
        wrap_with_desc(f, (id_world, world), world=world)
    test_caller.world = id_world
    test_caller.__name__ = 'test_%s_%s' % (f.__name__, id_world)
    add_to_module(test_caller, f.__module__)


def add_sensor_f(f, id_sensor):
    @istest
    def test_caller():
        sensor = get_sensor(id_sensor)
        wrap_with_desc(f, (id_sensor, sensor), sensor=sensor)
    test_caller.sensor = id_sensor
    test_caller.__name__ = 'test_%s_%s' % (f.__name__, id_sensor)
    add_to_module(test_caller, f.__module__)


def add_vehicle_f(f, id_vehicle):
    @istest
    def test_caller():
        vehicle = get_vehicle(id_vehicle)
        wrap_with_desc(f, (id_vehicle, vehicle), vehicle=vehicle)
    test_caller.vehicle = id_vehicle
    test_caller.__name__ = 'test_%s_%s' % (f.__name__, id_vehicle)
    add_to_module(test_caller, f.__module__)


def for_all_dynamics(f):
    ''' Decorator for dynamics tests. '''
    for id_dynamics in all_dynamics():
        add_dynamics_f(f, id_dynamics)


def for_all_worlds(f):
    for id_world in all_worlds():
        add_world_f(f, id_world)


def for_all_sensors(f):
    for id_sensor in all_sensors():
        add_sensor_f(f, id_sensor)


def for_all_vehicles(f):
    for id_vehicle in all_vehicles():
        add_vehicle_f(f, id_vehicle)


def for_all_world_vehicle_pairs(f):
    for id_world in all_worlds():
        for id_vehicle in all_vehicles():
            add_world_vehicle_f(f, id_world, id_vehicle)


def for_all_dynamics_sensor_pairs(f):
    for id_dynamics in all_dynamics():
        for id_sensor in all_sensors():
            add_pair_f(f, id_dynamics, id_sensor)


def wrap_with_desc(function, arguments, dynamics=None, world=None,
                   sensor=None, vehicle=None):
    ''' Calls function with arguments, and writes debug information
        if an exception is detected. '''

    try:
        function(*arguments)
    except:
        msg = ('Error detected when running test (%s); displaying debug info.'
               % function.__name__)
        if dynamics is not None:
            msg += '\ndynamics: %s' % dynamics
        # TODO: write other info
        logger.error(msg)
        raise
