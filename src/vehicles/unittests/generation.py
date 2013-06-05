
from comptests import comptests_for_all, comptests_for_all_pairs
from vehicles import (get_conftools_dynamics, get_conftools_vehicles,
    get_conftools_worlds, get_conftools_sensors, get_conftools_skins)

# 
# 
# # TODO: reintroduce this
# def wrap_with_desc(function, arguments, dynamics=None, world=None,
#                    sensor=None, vehicle=None):
#     ''' Calls function with arguments, and writes debug information
#         if an exception is detected. '''
# 
#     try:
#         function(*arguments)
#     except:
#         msg = ('Error detected when running test (%s); displaying debug info.'
#                % function.__name__)
#         if dynamics is not None:
#             msg += '\ndynamics: %s' % dynamics
#         # TODO: write other info
#         logger.error(msg)
#         raise


for_all_dynamics = comptests_for_all(get_conftools_dynamics())
for_all_vehicles = comptests_for_all(get_conftools_vehicles())
for_all_worlds = comptests_for_all(get_conftools_worlds())
for_all_sensors = comptests_for_all(get_conftools_sensors())
for_all_skins = comptests_for_all(get_conftools_skins())
for_all_world_vehicle_pairs = comptests_for_all_pairs(get_conftools_worlds(),
                                                      get_conftools_vehicles())

# 
# for_all_dynamics = fancy_test_decorator(lister=all_dynamics,
#             arguments=lambda x: (x, get_dynamics(x)),
#             attributes=lambda x: dict(dynamics=x))
# 
# for_all_worlds = fancy_test_decorator(lister=all_worlds,
#             arguments=lambda x: (x, get_world(x)),
#             attributes=lambda x: dict(world=x))
# 
# for_all_vehicles = fancy_test_decorator(lister=all_vehicles,
#             arguments=lambda x: (x, get_vehicle(x)),
#             attributes=lambda x: dict(vehicle=x))
# 
# for_all_sensors = fancy_test_decorator(lister=all_sensors,
#             arguments=lambda x: (x, get_sensor(x)),
#             attributes=lambda x: dict(sensor=x))
# 
# for_all_skins = fancy_test_decorator(lister=all_skins,
#             arguments=lambda x: (x, get_skin(x)),
#             attributes=lambda x: dict(skin=x))
# 
# for_all_world_vehicle_pairs = fancy_test_decorator(
#             lister=lambda: itertools.product(all_worlds(), all_vehicles()),
#             arguments=lambda (x, y): (x, get_world(x), y, get_vehicle(y)),
#             attributes=lambda (x, y): dict(world=x, vehicle=y),
#                 naming=lambda (a, b): '%s-%s' % (a, b))

