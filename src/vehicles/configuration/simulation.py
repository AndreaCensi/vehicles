from . import check_valid_vehicle_config, check_valid_world_config
from conf_tools import check_has_exactly_one, wrap_check


def check_valid_simulation_config(x):
    vehicles_alt = [('vehicle', dict), ('id_vehicle', str)]
    check_has_exactly_one(x, vehicles_alt)

    if 'vehicle' in x:
        wrap_check(x, 'checking "vehicle" field',
                   check_valid_vehicle_config, x['vehicle'])

    world_alt = [('world', dict), ('id_world', str)]
    check_has_exactly_one(x, world_alt)

    if 'world' in x:
        wrap_check(x, 'checking "world" field',
                   check_valid_world_config, x['world'])
