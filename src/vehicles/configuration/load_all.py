from .. import logger
from . import load_configuration_entries
from vehicles.configuration.checks import check_valid_vehicle_config, \
    check_valid_sensor_config, check_valid_dynamics_config, \
    check_valid_world_config

class Configuration:
    loaded = False
    dynamics = {}
    sensors = {}
    vehicles = {}
    worlds = {}




def load_configuration(directory=None,
                       pattern_dynamics='*.dynamics.yaml',
                       pattern_sensors='*.sensors.yaml',
                       pattern_vehicles='*.vehicles.yaml',
                       pattern_worlds='*.worlds.yaml'):
    ''' 
        Loads all configuration files from the directory. 
        If directory is not specified, it uses the default directory. 
    '''
    Configuration.loaded = True
    
    if directory is None:
        from pkg_resources import resource_filename #@UnresolvedImport
        directory = resource_filename("vehicles", "configs")
        
    logger.info('Loading configuration from %r' % directory)

    def merge(original, new):
        for x in new:
            if x in original and new[x]['filename'] != original[x]['filename']:
                msg = ('Entry %r (%r) already present in %r.' % 
                       (x, new[x]['filename'], original[x]['filename'])) 
                raise Exception(msg)
        original.update(new)
    
    worlds = load_configuration_entries(directory,
                                        pattern=pattern_worlds,
                                        check_entry=check_valid_world_config)
    merge(Configuration.worlds, worlds)

    dynamics = load_configuration_entries(directory,
                                        pattern=pattern_dynamics,
                                        check_entry=check_valid_dynamics_config)
    merge(Configuration.dynamics, dynamics)

    sensors = load_configuration_entries(directory,
                                        pattern=pattern_sensors,
                                        check_entry=check_valid_sensor_config)
    merge(Configuration.sensors, sensors)
        
    vehicles = load_configuration_entries(directory,
                                        pattern=pattern_vehicles,
                                        check_entry=check_valid_vehicle_config)
    merge(Configuration.vehicles, vehicles)
    
    logger.debug('Found %5d worlds.' % len(Configuration.worlds))
    logger.debug('Found %5d vehicles.' % len(Configuration.vehicles))
    logger.debug('Found %5d dynamics.' % len(Configuration.dynamics))
    logger.debug('Found %5d sensors.' % len(Configuration.sensors))
                               

    add_blind_vehicles()
    
def add_blind_vehicles():
    for id_dynamics in Configuration.dynamics:
        id_vehicle = '_blind-%s' % id_dynamics
        vehicle = {
           'id': id_vehicle,
           'desc': 'Blind vehicle with dynamics %s.' % id_dynamics,
           'id_dynamics': id_dynamics,
           'sensors': [{'id_sensor': 'random_5', 'pose': [0, 0, 0]}],
           'radius': 0.5
        }
        check_valid_vehicle_config(vehicle)
        Configuration.vehicles[id_vehicle] = vehicle
    
    
    

