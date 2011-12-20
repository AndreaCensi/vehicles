from . import (for_all_sensors, for_all_vehicles, for_all_dynamics,
    for_all_worlds)

# TODO: check basic things


@for_all_vehicles
def check_vehicle_instance(id_vehicle, vehicle):
    pass


@for_all_sensors
def check_sensor_instance(id_sensor, sensor):
    pass


@for_all_worlds
def check_world_instance(id_world, world):
    pass


@for_all_dynamics
def check_dynamics_instance(id_dynamics, dynamics):
    pass

