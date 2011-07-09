from . import Configuration, load_configuration, instantiate_spec
from .. import logger
from contracts.interface import describe_value
from pprint import pformat
from geometry.manifolds import SE3, SE2
from geometry.poses import SE3_from_SE2


def instance_all():
    ''' 
        Instantiates all dynamics, worlds, sensors and vehicles. 
        It creates an entry named 'instance' in the dictionaries in 
        the class Configuration.
        
        This is useful for debugging that everything works fine.
    '''

    for name, entry in Configuration.dynamics.items():
        entry['instance'] = instance_dynamics(name)

    for name, entry in Configuration.sensors.items():
        entry['instance'] = instance_sensor(name)
        
    for name, entry in Configuration.worlds.items():
        entry['instance'] = instance_world(name)

    for name, entry in Configuration.vehicles.items():
        entry['instance'] = instance_vehicle(name)


def check_type(entry, type, obtained):
    if not isinstance(obtained, type):
        msg = 'Error in instantiating code spec:\n\t%s\n' % entry['code']
        msg += 'I expected a %s, got %s' % (type, describe_value(obtained)) 
        raise Exception(msg)

def instance_dynamics(id_dynamics):
    from vehicles_dynamics import Dynamics
    if not Configuration.loaded:
        load_configuration()
        
    if not id_dynamics in Configuration.dynamics:
        raise Exception('No dynamics %r known.' % id_dynamics)
    entry = Configuration.dynamics[id_dynamics]
    try:
        instance = instantiate_spec(entry['code'])
    except:
        logger.error('Error while trying to instantiate dynamics %r. Entry:\n%s' 
                     % (id_dynamics, pformat(entry)))
        raise
    check_type(entry, Dynamics, instance)
    return instance

def instance_sensor(id_sensor):
    from ..interfaces import VehicleSensor
    
    if not Configuration.loaded:
        load_configuration()

    if not id_sensor in Configuration.sensors:
        raise Exception('No sensor %r known.' % id_sensor)
    entry = Configuration.sensors[id_sensor]
    try:
        instance = instantiate_spec(entry['code'])
        check_type(entry, VehicleSensor, instance)
    except:
        logger.error('Error while trying to instantiate sensor %r. Entry:\n%s' 
                     % (id_sensor, pformat(entry)))
        raise
    return instance
    
def instance_world(id_world):
    from ..interfaces import World
    if not Configuration.loaded:
        load_configuration()

    if not id_world in Configuration.worlds:
        raise Exception('No world %r known.' % id_world)
    entry = Configuration.worlds[id_world]
    try:
        instance = instantiate_spec(entry['code'])
        check_type(entry, World, instance)
    except:
        logger.error('Error while trying to instantiate world %r. Entry:\n%s' 
                     % (id_world, pformat(entry)))
        raise
    return instance

def instance_vehicle(id_vehicle):
    from ..simulation import Vehicle
    
    if not Configuration.loaded:
        load_configuration()

    
    if not id_vehicle in Configuration.vehicles:
        raise Exception('No vehicle %r known.' % id_vehicle)
    entry = Configuration.vehicles[id_vehicle]
    try:
        type_dynamics = entry['dynamics']
        dynamics = instance_dynamics(type_dynamics)
        sensors = entry['sensors']
        vehicle = Vehicle()
        vehicle.add_dynamics(type_dynamics, dynamics)
        for sensor in sensors:
            sensor_type = sensor['type']
            pose = SE3_from_SE2(SE2.from_yaml(sensor['pose']))
            joint = sensor.get('joint', 0)
            sensor_instance = instance_sensor(sensor_type)
            vehicle.add_sensor(id_sensor=sensor_type,
                               sensor=sensor_instance, pose=pose, joint=joint)
        return vehicle
    except:
        logger.error('Error while trying to instantiate vehicle %r. Entry:\n%s' 
                     % (id_vehicle, pformat(entry)))
        raise
