from comptests import (comptests_for_all, comptests_for_all_dynamic, 
    comptests_for_all_pairs)
from vehicles import (get_conftools_dynamics, get_conftools_sensors, 
    get_conftools_skins, get_conftools_vehicles, get_conftools_worlds)

for_all_dynamics = comptests_for_all(get_conftools_dynamics())
for_all_vehicles = comptests_for_all(get_conftools_vehicles())
for_all_vehicles_context = comptests_for_all_dynamic(get_conftools_vehicles())
for_all_worlds = comptests_for_all(get_conftools_worlds())
for_all_sensors = comptests_for_all(get_conftools_sensors())
for_all_skins = comptests_for_all(get_conftools_skins())
for_all_world_vehicle_pairs = comptests_for_all_pairs(get_conftools_worlds(),
                                                      get_conftools_vehicles())

