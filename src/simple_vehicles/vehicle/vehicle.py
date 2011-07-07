from collections import namedtuple


class Vehicle:
    
    def __init__(self, dynamics):
        self.num_sensels = 0
        self.sensors = []
        self.id_sensors = None
        self.id_dynamics = None # XXX
        self.dynamics = None
        
    def add_dynamics(self, id_dynamics, dynamics):
        assert self.dynamics is None, 'not sure if this will be implemented'
        self.dynamics = dynamics
        self.commands_spec = dynamics.commands_spec
        self.id_dynamics = id_dynamics
    
    AttachedSensor = namedtuple('AttachedSensor', 'sensor pose joint')
    def add_sensor(self, id_sensor, sensor, pose, joint):
        attached = Vehicle.AttachedSensor(sensor, pose, joint)
        self.sensors.append(attached)
        self.num_sensels += attached.sensor.num_sensels
        if not self.id_sensors:
            self.id_sensors = id_sensor
        else:
            self.id_sensors += '+%s' % id_sensor
