from . import (all_dynamics, get_dynamics, logger, get_sensor, all_sensors,
    all_worlds, get_vehicle, all_vehicles, get_world, get_skin, all_skins)
from .utils import fancy_test_decorator
import itertools


# TODO: reintroduce this
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


for_all_dynamics = fancy_test_decorator(lister=all_dynamics,
            arguments=lambda x: (x, get_dynamics(x)),
            attributes=lambda x: dict(dynamics=x))

for_all_worlds = fancy_test_decorator(lister=all_worlds,
            arguments=lambda x: (x, get_world(x)),
            attributes=lambda x: dict(world=x))

for_all_vehicles = fancy_test_decorator(lister=all_vehicles,
            arguments=lambda x: (x, get_vehicle(x)),
            attributes=lambda x: dict(vehicle=x))

for_all_sensors = fancy_test_decorator(lister=all_sensors,
            arguments=lambda x: (x, get_sensor(x)),
            attributes=lambda x: dict(sensor=x))

for_all_skins = fancy_test_decorator(lister=all_skins,
            arguments=lambda x: (x, get_skin(x)),
            attributes=lambda x: dict(skin=x))

for_all_world_vehicle_pairs = fancy_test_decorator(
            lister=lambda: itertools.product(all_worlds(), all_vehicles()),
            arguments=lambda (x, y): (x, get_world(x), y, get_vehicle(y)),
            attributes=lambda (x, y): dict(world=x, vehicle=y),
                naming=lambda (a, b): '%s-%s' % (a, b))

