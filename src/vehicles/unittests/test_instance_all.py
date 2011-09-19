from vehicles import VehiclesConfig


def check_one(method, x):
    method(x)
    
def test_instance_all():
    VehiclesConfig.load()
    
    for name, entry in VehiclesConfig.dynamics.items():
        yield VehiclesConfig.dynamics.instance, name #@UndefinedVariable
        
    for name, entry in VehiclesConfig.sensors.items():
        yield  VehiclesConfig.sensors.instance, name #@UndefinedVariable
        
    for name, entry in VehiclesConfig.worlds.items():
        yield  VehiclesConfig.worlds.instance, name #@UndefinedVariable

    for name, entry in VehiclesConfig.vehicles.items():
        yield  VehiclesConfig.vehicles.instance, name #@UndefinedVariable


    
def instance_all():
    ''' 
        Instantiates all dynamics, worlds, sensors and vehicles. 
        It creates an entry named 'instance' in the dictionaries in 
        the class Configuration.
        
        This is useful for debugging that everything works fine.
    '''
    
    VehiclesConfig.load()

    for name, entry in VehiclesConfig.dynamics.items():
        entry['instance'] = VehiclesConfig.dynamics.instance(name) #@UndefinedVariable
        
    for name, entry in VehiclesConfig.sensors.items():
        entry['instance'] = VehiclesConfig.sensors.instance(name) #@UndefinedVariable
        
    for name, entry in VehiclesConfig.worlds.items():
        entry['instance'] = VehiclesConfig.worlds.instance(name) #@UndefinedVariable

    for name, entry in VehiclesConfig.vehicles.items():
        entry['instance'] = VehiclesConfig.vehicles.instance(name) #@UndefinedVariable


#
#    for name, entry in VehiclesConfig.dynamics.items():
#        entry['instance'] = instance_dynamics(name)
#
#    for name, entry in VehiclesConfig.sensors.items():
#        entry['instance'] = instance_sensor(name)
#        
#    for name, entry in VehiclesConfig.worlds.items():
#        entry['instance'] = instance_world(name)
#
#    for name, entry in VehiclesConfig.vehicles.items():
#        entry['instance'] = instance_vehicle(name)
