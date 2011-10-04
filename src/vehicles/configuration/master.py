from . import instance_vehicle_spec
from ..interfaces import VehicleSensor, World
from . import (check_valid_vehicle_config, check_valid_world_config,
    check_valid_dynamics_config, check_valid_sensor_config)
from conf_tools import ConfigMaster, GenericInstance
from vehicles_dynamics import Dynamics

class VehiclesConfigMaster(ConfigMaster):
    
    def __init__(self):
        ConfigMaster.__init__(self)

        self.add_class('vehicles', '*.vehicles.yaml',
                        check_valid_vehicle_config, instance_vehicle_spec)
        self.add_class('worlds', '*.worlds.yaml', check_valid_world_config,
                       GenericInstance(World))
        self.add_class('dynamics', '*.dynamics.yaml', check_valid_dynamics_config,
                       GenericInstance(Dynamics))
        self.add_class('sensors', '*.sensors.yaml', check_valid_sensor_config,
                       GenericInstance(VehicleSensor))
        
        self.vehicles = self.specs['vehicles']
        self.worlds = self.specs['worlds']
        self.dynamics = self.specs['dynamics']
        self.sensors = self.specs['sensors']
        
 
    def get_default_dir(self):
        from pkg_resources import resource_filename #@UnresolvedImport
        directory = resource_filename("vehicles", "configs")
        return directory
    
VehiclesConfig = VehiclesConfigMaster()
    
