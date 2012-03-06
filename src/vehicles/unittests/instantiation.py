from vehicles.configuration.master import VehiclesConfig


def make_sure_vehicles_config_loaded():
    if not VehiclesConfig.loaded:
#        v = VehiclesConstants.TEST_ADDITIONAL_CONFIG_DIR_ENV
#        if v in os.environ:
#            for dirname in os.environ[v].split(':'):
#                logger.info('Using additional dir %r.' % dirname)
#                VehiclesConfig.load(dirname)
#        else:
#            logger.info('Use env var %s to add more config dirs.' % v)
        VehiclesConfig.load()


def all_dynamics():
    ''' Returns a list of all test dynamics IDs. '''
    make_sure_vehicles_config_loaded()
    dynamics = list(VehiclesConfig.dynamics.keys())
    if not dynamics:
        raise Exception('No dynamics defined in this configuration.')
    return dynamics


def all_sensors():
    ''' Returns a list of all test sensors IDs. '''
    make_sure_vehicles_config_loaded()
    sensors = list(VehiclesConfig.sensors.keys())
    if not sensors:
        raise Exception('No sensors defined in this configuration.')
    return sensors


def all_worlds():
    ''' Returns a list of all test world IDs. '''
    make_sure_vehicles_config_loaded()
    worlds = list(VehiclesConfig.worlds.keys())
    if not worlds:
        raise Exception('No worlds defined in this configuration.')
    return worlds


def all_vehicles():
    ''' Returns a list of all test vehicle IDs. '''
    make_sure_vehicles_config_loaded()
    vehicles = list(VehiclesConfig.vehicles.keys())
    if not vehicles:
        raise Exception('No vehicles defined in this configuration.')
    return vehicles


def all_skins():
    ''' Returns a list of all test skin IDs. '''
    make_sure_vehicles_config_loaded()
    skins = list(VehiclesConfig.specs['skins'].keys())
    if not skins:
        raise Exception('No skins defined in this configuration.')
    return skins


def get_dynamics(id_dynamics):
    make_sure_vehicles_config_loaded()
    return VehiclesConfig.dynamics.instance(id_dynamics)  # @UndefinedVariable


def get_skin(id_skin):
    make_sure_vehicles_config_loaded()
    return VehiclesConfig.specs['skins'].instance(id_skin)


def get_world(id_world):
    make_sure_vehicles_config_loaded()
    return VehiclesConfig.worlds.instance(id_world)  # @UndefinedVariable


def get_vehicle(id_vehicle):
    make_sure_vehicles_config_loaded()
    return VehiclesConfig.vehicles.instance(id_vehicle)  # @UndefinedVariable


def get_sensor(id_sensor):
    make_sure_vehicles_config_loaded()
    return VehiclesConfig.sensors.instance(id_sensor)  # @UndefinedVariable
