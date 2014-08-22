from conf_tools import ConfigMaster, ObjectSpec
from contracts import contract


__all__ = [
   'get_conftools_dynamics',
   'get_conftools_sensors',
   'get_conftools_skins',
   'get_conftools_vehicles',
   'get_conftools_worlds',
   'get_vehicles_config',
]

@contract(returns=ObjectSpec)
def get_conftools_vehicles():
    return get_vehicles_config().vehicles

@contract(returns=ObjectSpec)
def get_conftools_worlds():
    return get_vehicles_config().worlds

@contract(returns=ObjectSpec)
def get_conftools_dynamics():
    return get_vehicles_config().dynamics

@contract(returns=ObjectSpec)
def get_conftools_sensors():
    return get_vehicles_config().sensors

@contract(returns=ObjectSpec)
def get_conftools_skins():
    return get_vehicles_config().skins

class VehiclesConfigMaster(ConfigMaster):

    def __init__(self):
        ConfigMaster.__init__(self, 'Vehicles')
        from vehicles import (Dynamics, VehicleSensor, VehicleSkin, 
                              World, Vehicle)
        acg = self.add_class_generic 
        acg('worlds', '*.worlds.yaml', World)
        acg('dynamics', '*.dynamics.yaml', Dynamics)
        acg('sensors', '*.sensors.yaml', VehicleSensor)
        acg('skins', '*.skins.yaml', VehicleSkin)
        acg('vehicles', '*.vehicles.yaml', Vehicle)
 
    def get_default_dir(self):
        return "vehicles.configs"


get_vehicles_config = VehiclesConfigMaster.get_singleton

