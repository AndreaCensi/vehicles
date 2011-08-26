from . import Raytracer, contract, np
from ..interfaces import VehicleSensor
from geometry import SE2_project_from_SE3
from ..configuration import instantiate_spec

class Rangefinder(VehicleSensor, Raytracer):

    @contract(directions='seq[>0](number)', invalid='number')
    def __init__(self, directions, invalid=10, min_range=0.2, max_range=10,
                 noise=None):
        ''' 
            :param directions: array of orientations
            :param invalid: value for invalid data (i.e., out of range) 
        '''
        self.invalid = invalid
        self.min_range = min_range
        self.max_range = max_range
        
        if noise is None:
            self.noise = None
        else:
            self.noise = instantiate_spec(noise)
        
        VehicleSensor.__init__(self, len(directions))
        Raytracer.__init__(self, directions)
    
    def _compute_observations(self, pose):
        pose = SE2_project_from_SE3(pose)
        data = self.raytracing(pose)
        readings = data['readings']
        invalid = np.logical_not(data['valid'])
        readings[:] = np.minimum(readings, self.max_range)
        readings[:] = np.maximum(readings, self.min_range)
        
        if self.noise is not None:
            readings = self.noise.filter(readings)

        readings[invalid] = self.invalid
        
        sensels = (readings - self.min_range) / (self.max_range - self.min_range)
        data[VehicleSensor.SENSELS] = sensels
        return data

    def set_world_primitives(self, primitives):
        Raytracer.set_world_primitives(self, primitives)

    
class RangefinderUniform(Rangefinder):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, noise=None):
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
        Rangefinder.__init__(self, directions=directions, noise=noise)
        

        
