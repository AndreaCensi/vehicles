from . import (check_valid_vehicle_config, check_valid_world_config, logger,
    check_valid_dynamics_config, check_valid_sensor_config, check_valid_skin_config,
     instance_vehicle_spec)
from .. import VehiclesConstants
from ..interfaces import VehicleSensor, World, VehicleSkin, Dynamics
from conf_tools import ConfigMaster, GenericInstance
import os


class VehiclesConfigMaster(ConfigMaster):

    def __init__(self):
        ConfigMaster.__init__(self, 'Vehicles')

        self.add_class('vehicles', '*.vehicles.yaml',
                        check_valid_vehicle_config, instance_vehicle_spec)

        self.add_class('worlds', '*.worlds.yaml', check_valid_world_config,
                       GenericInstance(World))

        self.add_class('dynamics', '*.dynamics.yaml',
                       check_valid_dynamics_config,
                       GenericInstance(Dynamics))

        self.add_class('sensors', '*.sensors.yaml', check_valid_sensor_config,
                       GenericInstance(VehicleSensor))

        self.add_class('skins', '*.skins.yaml',
                       check_valid_skin_config,
                       GenericInstance(VehicleSkin))

        self.vehicles = self.specs['vehicles']
        self.worlds = self.specs['worlds']
        self.dynamics = self.specs['dynamics']
        self.sensors = self.specs['sensors']
        self.skins = self.specs['skins']

        v = VehiclesConstants.TEST_ADDITIONAL_CONFIG_DIR_ENV
        if v in os.environ:
            for dirname in os.environ[v].split(':'):
                if dirname == 'default':
                    logger.info('Using default config dir.')
                    self.load()
                else:
                    logger.info('Using additional dir %r.' % dirname)
                    self.load(dirname)

        else:
            #logger.info('Use env var %s to add more config dirs.' % v)
            pass

    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        directory = resource_filename("vehicles", "configs")
        return directory

VehiclesConfig = VehiclesConfigMaster()
