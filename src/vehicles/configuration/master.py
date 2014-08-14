from .vehicles_c import check_valid_vehicle_config, instance_vehicle_spec
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
    """ Returns the object responsible for instancing DiffeoActionDistance. """
    return get_vehicles_config().vehicles

@contract(returns=ObjectSpec)
def get_conftools_worlds():
    """ Returns the object responsible for instancing DiffeoActionDistance. """
    return get_vehicles_config().worlds

@contract(returns=ObjectSpec)
def get_conftools_dynamics():
    """ Returns the object responsible for instancing DiffeoActionDistance. """
    return get_vehicles_config().dynamics

@contract(returns=ObjectSpec)
def get_conftools_sensors():
    """ Returns the object responsible for instancing DiffeoActionDistance. """
    return get_vehicles_config().sensors

@contract(returns=ObjectSpec)
def get_conftools_skins():
    """ Returns the object responsible for instancing DiffeoActionDistance. """
    return get_vehicles_config().skins

class VehiclesConfigMaster(ConfigMaster):

    def __init__(self):
        ConfigMaster.__init__(self, 'Vehicles')
        from vehicles import Dynamics, VehicleSensor,  VehicleSkin, World
        self.add_class('vehicles', '*.vehicles.yaml', 
            check_valid_vehicle_config, instance_vehicle_spec)

        acg = self.add_class_generic 
        acg('worlds', '*.worlds.yaml', World)
        acg('dynamics', '*.dynamics.yaml', Dynamics)
        acg('sensors', '*.sensors.yaml', VehicleSensor)
        acg('skins', '*.skins.yaml', VehicleSkin)
 
    def get_default_dir(self):
        return "vehicles.configs"


get_vehicles_config = VehiclesConfigMaster.get_singleton

