from .. import logger
from simple_vehicles.loading.utils import load_configuration_entries

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
        directory = resource_filename("simple_vehicles", "configs")
        
    logger.info('Loading configuration from %r' % directory)

    def merge(original, new):
        for x in new:
            if x in original:
                msg = ('Entry %r (%r) already present in %r.' % 
                       (x, new[x]['filename'], original[x]['filename'])) 
                raise Exception(msg)
        original.update(new)
    
    worlds = load_configuration_entries(directory,
                                        pattern=pattern_worlds,
                                        required_fields=['id', 'desc', 'code'],
                                        optional_fields=[])
    merge(Configuration.worlds, worlds)

    dynamics = load_configuration_entries(directory,
                                        pattern=pattern_dynamics,
                                        required_fields=['id', 'desc', 'code'],
                                        optional_fields=[],
                                        check_not_allowed=False)
    merge(Configuration.dynamics, dynamics)

    sensors = load_configuration_entries(directory,
                                        pattern=pattern_sensors,
                                        required_fields=['id', 'desc', 'code'],
                                        optional_fields=[])
    merge(Configuration.sensors, sensors)
        
    vehicles = load_configuration_entries(directory,
                                        pattern=pattern_vehicles,
                                        required_fields=['id', 'desc', 'sensors',
                                                         'dynamics'],
                                        optional_fields=['body'])
    merge(Configuration.vehicles, vehicles)
    
    logger.debug('Found %5d worlds.' % len(Configuration.worlds))
    logger.debug('Found %5d vehicles.' % len(Configuration.vehicles))
    logger.debug('Found %5d dynamics.' % len(Configuration.dynamics))
    logger.debug('Found %5d sensors.' % len(Configuration.sensors))
                               
def load_vehicle_from_id(id_robot):
    if not Configuration.loaded():
        load_configuration()
        
    pass

def load_world_from_id(id_world):
    if not Configuration.loaded():
        load_configuration()
        
