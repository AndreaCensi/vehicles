from vehicles import (instance_vehicle, instance_world, instance_sensor,
    instance_dynamics, load_configuration, Configuration)


def test_configuration():
    ''' Simply load the configuration. '''    
    load_configuration()
     


def test_instantiation_single():
    
    for name in Configuration.dynamics:
        yield instance_dynamics, name

    for name in Configuration.sensors:
        yield instance_sensor, name
        
    for name  in Configuration.worlds:
        yield instance_world, name

    for name in Configuration.vehicles:
        yield instance_vehicle, name
