from . import Raytracer, contract, np
from ..interfaces import VehicleSensor

class Rangefinder(VehicleSensor, Raytracer):

    @contract(directions='seq[>0](number)', invalid='number')
    def __init__(self, directions, invalid=0):
        ''' directions: array of orientations
            invalid: value for invalid data (infinity) 
        '''
        self.invalid = invalid
        VehicleSensor.__init__(self, len(directions))
        Raytracer.__init__(self, directions)
    
    def _compute_observations(self, pose):
        data = self.raytracing(pose)
        readings = data['readings']
        invalid = np.logical_not(data['valid'])
        readings[invalid] = self.invalid
        data[VehicleSensor.SENSELS] = readings
        return data

    def set_world(self, world, updated):
        Raytracer.set_world(self, world, updated)

    
class RangefinderUniform(Rangefinder):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels):
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
        Rangefinder.__init__(self, directions)
        

        
