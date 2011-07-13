from . import Configuration, load_configuration, instantiate_spec
from .. import logger
from contracts.interface import describe_value
from geometry import SE2, SE3_from_SE2
from pprint import pformat
from vehicles.configuration.checks import check_valid_vehicle_config
from copy import deepcopy



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
    if not Configuration.loaded:
        load_configuration()
        
    if not id_dynamics in Configuration.dynamics:
        raise Exception('No dynamics %r known.' % id_dynamics)
    entry = Configuration.dynamics[id_dynamics]    
    return instance_dynamics_spec(entry)

def instance_dynamics_spec(entry):
    from vehicles_dynamics import Dynamics
    try:
        instance = instantiate_spec(entry['code'])
    except:
        logger.error('Error while trying to instantiate dynamics. Entry:\n%s' 
                     % (pformat(entry)))
        raise
    check_type(entry, Dynamics, instance)
    return instance

def instance_sensor(id_sensor):
    
    if not Configuration.loaded:
        load_configuration()

    if not id_sensor in Configuration.sensors:
        raise Exception('No sensor %r known.' % id_sensor)
    entry = Configuration.sensors[id_sensor]
    return instance_sensor_spec(entry)

def instance_sensor_spec(entry):
    from ..interfaces import VehicleSensor
    
    try:
        instance = instantiate_spec(entry['code'])
        check_type(entry, VehicleSensor, instance)
    except:
        logger.error('Error while trying to instantiate sensor. Entry:\n%s' 
                     % (pformat(entry)))
        raise
    return instance
    
def instance_world(id_world):
    
    if not Configuration.loaded:
        load_configuration()

    if not id_world in Configuration.worlds:
        raise Exception('No world %r known.' % id_world)
    entry = Configuration.worlds[id_world]
    return instance_world_spec(entry)

def instance_world_spec(entry):
    from ..interfaces import World
    try:
        instance = instantiate_spec(entry['code'])
        check_type(entry, World, instance)
    except:
        logger.error('Error while trying to instantiate world. Entry:\n%s' 
                     % (pformat(entry)))
        raise
    return instance

def instance_vehicle(id_vehicle):    
    if not Configuration.loaded:
        load_configuration()

    if not id_vehicle in Configuration.vehicles:
        raise Exception('No vehicle %r known.' % id_vehicle)
    entry = Configuration.vehicles[id_vehicle]
    return instance_vehicle_spec(entry)

    
def instance_vehicle_spec(entry):
    from ..simulation import Vehicle

    check_valid_vehicle_config(entry)
    try:
        if 'id_dynamics' in entry:
            id_dynamics = entry['id_dynamics']
            dynamics = instance_dynamics(id_dynamics)
        else:
            id_dynamics = entry['dynamics']['id']
            dynamics = instance_dynamics_spec(entry['dynamics'])
            
        sensors = entry['sensors']
        vehicle = Vehicle()
        vehicle.add_dynamics(id_dynamics, dynamics)
        for sensor in sensors:
            if 'id_sensor' in sensor:
                id_sensor = sensor['id_sensor'] 
                sensor_instance = instance_sensor(id_sensor)
            else:
                id_sensor = sensor['sensor']['id'] 
                sensor_instance = instance_sensor_spec(sensor['sensor'])
                
            pose = SE3_from_SE2(SE2.from_yaml(sensor['pose']))
            joint = sensor.get('joint', 0)
            vehicle.add_sensor(id_sensor=id_sensor,
                               sensor=sensor_instance, pose=pose, joint=joint)
        return vehicle
    except:
        logger.error('Error while trying to instantiate vehicle. Entry:\n%s' 
                     % (pformat(entry)))
        raise

def dereference_vehicle_spec(x):
    ''' substitutes all references; modifies x. '''
    check_valid_vehicle_config(x)
    if 'id_dynamics' in x:
        id_dynamics = x.pop('id_dynamics')
        x['dynamics'] = deepcopy(Configuration.dynamics[id_dynamics])
    for s in x['sensors']:
        if 'id_sensor' in s:
            id_sensor = s.pop('id_sensor')
            s['sensor'] = deepcopy(Configuration.sensors[id_sensor])
    check_valid_vehicle_config(x)


