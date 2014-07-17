
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
