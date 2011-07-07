from vehicles_dynamics.interface import Dynamics
from collections import namedtuple


class Vehicle:
    
    Dynamics
    def __init__(self, dynamics):
        self.dynamics = dynamics
        self.commands_spec = dynamics.commands_spec
        self.num_sensels = 0
        self.sensors = []
        
    AttachedSensor = namedtuple('AttachedSensor', 'sensor pose joint')
    def add_sensor(self, sensor, pose, joint):
        attached = Vehicle.AttachedSensor(sensor, pose, joint)
        self.sensors.append(attached)
        self.num_sensels += attached.sensor.num_sensels
        
