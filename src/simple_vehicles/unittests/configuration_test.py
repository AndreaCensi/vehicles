from ..loading import load_configuration, Configuration
from simple_vehicles.loading.instance_all import instance_all, instance_vehicle, \
    instance_world, instance_sensor, instance_dynamics


def test_configuration():
    ''' Simply load the configuration. '''    
    load_configuration()
    

#def test_instantiation():
#    if not Configuration.loaded:
#        load_configuration()
#    instance_all()


def test_instantiation_single():
    
    for name in Configuration.dynamics:
        yield instance_dynamics, name

    for name in Configuration.sensors:
        yield instance_sensor, name
        
    for name  in Configuration.worlds:
        yield instance_world, name

    for name in Configuration.vehicles:
        yield instance_vehicle, name
