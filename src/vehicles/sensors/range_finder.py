from . import Raytracer, contract, np
from ..interfaces import VehicleSensor
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3
from numpy.testing.utils import assert_allclose


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
        
        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else 
                          instantiate_spec(self.noise_spec))
        
        spec = {
            'desc': 'Range-finder',
            'shape': [len(directions)],
            'format': 'C',
            'range': [0, +1],
            'extra': {'directions': directions.tolist(),
                      'noise': noise }
        }
        VehicleSensor.__init__(self, spec)
        Raytracer.__init__(self, directions)
    
    def to_yaml(self):
        return {'type': 'Rangefinder',
                'noise_spec': self.noise_spec,
                'invalid': self.invalid,
                'min_range': self.min_range,
                'max_range': self.max_range,
                'directions': self.directions.tolist()}
    
    def _compute_observations(self, pose):
        pose = SE2_project_from_SE3(pose)
        data = self.raytracing(pose)
        readings = data['readings']
        invalid = np.logical_not(data['valid'])
        readings[:] = np.minimum(readings, self.max_range)
        readings[:] = np.maximum(readings, self.min_range)
        
        if self.noise is not None:
            readings = self.noise.filter(readings)
            readings = np.minimum(self.max_range, readings)
            readings = np.maximum(self.min_range, readings)

        readings[invalid] = self.invalid
        
        sensels = ((readings - self.min_range) / 
                   (self.max_range - self.min_range))
        data[VehicleSensor.SENSELS] = sensels
        return data

    def set_world_primitives(self, primitives):
        Raytracer.set_world_primitives(self, primitives)

    
class RangefinderUniform(Rangefinder):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, noise=None):
        directions = get_uniform_directions(fov_deg, num_sensels)
        Rangefinder.__init__(self, directions=directions, noise=noise)
        

def get_uniform_directions(fov_deg, num_sensels):
    if fov_deg == 360:
        ray_dist = np.pi / (num_sensels - 1)
        directions = np.linspace(-np.pi + ray_dist / 2,
                                 + np.pi - ray_dist / 2, num_sensels)
        assert_allclose(directions[-1] - directions[0], 2 * np.pi - ray_dist)
    else:
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
    
        assert_allclose(directions[-1] - directions[0], fov_rad)
    assert len(directions) == num_sensels
    return directions
    
