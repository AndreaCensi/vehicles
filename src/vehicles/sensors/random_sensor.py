from ..interfaces import VehicleSensor
from contracts import contract
import numpy as np

class RandomSensor(VehicleSensor):

    @contract(num_sensels='>0')    
    def __init__(self, num_sensels):
        VehicleSensor.__init__(self, num_sensels)
        self.num_sensels = num_sensels
        
    def _compute_observations(self, pose):
        values = np.random.rand(self.num_sensels)
        data = {}
        data[VehicleSensor.SENSELS] = values
        return data

    def set_world_primitives(self, primitives):
        pass
