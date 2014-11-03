from contracts import contract

from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3
import numpy as np
from vehicles import VehicleSensor, VehiclesConstants

from .textured_raytracer import TexturedRaytracer
from .smoother import Smoother2


__all__ = [
    'PhotoreceptorsSmooth',
    'PhotoreceptorsSmoothUniform',
    'PhotoreceptorsSmoothRandom',
    'PhotoreceptorsPerturb',
]

    
class PhotoreceptorsSmooth(VehicleSensor, TexturedRaytracer):
    """ Implements  """

    @contract(directions='seq[>0](number)', spatial_sigma_deg='>=0')
    def __init__(self, directions, spatial_sigma_deg, upsample=7,
                 noise=None, invalid=0.5, d2=None):
        if upsample< 5:
            print('warning: upsample %d is small' % upsample)
        
        self.invalid = invalid
        self.invalid_range = 100 # xxx

        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else
                          instantiate_spec(self.noise_spec))

        #d = np.array(directions)
        #print d[:-1] - d[1:]
        self.smoother = Smoother2(directions=directions, 
                                 spatial_sigma_deg=spatial_sigma_deg, 
                                 upsample=upsample, d2=d2)
                                 
        self.directions_o = np.array(directions)
        self.directions2 = self.smoother.get_new_directions()
        spec = {
            'desc': 'Photoreceptors',
            'shape': [len(directions)],
            'format': 'C',
            'range': [0, +1],
            'extra': {'directions': self.directions_o.tolist(),
                      'noise': self.noise_spec,
                      'spatial_sigma_deg': spatial_sigma_deg},
        }

        VehicleSensor.__init__(self, spec)
        TexturedRaytracer.__init__(self, self.directions2)

    def to_yaml(self):
        return {'type': VehiclesConstants.SENSOR_TYPE_PHOTORECEPTORS,
                'noise': self.noise_spec,
                'invalid': self.invalid,
                'directions': self.directions_o.tolist()}

    def _compute_observations(self, pose):
        pose = SE2_project_from_SE3(pose)
        data = self.raytracing(pose)
        luminance = data['luminance']
        valid = data['valid']
        readings = data['readings']
        invalid = np.logical_not(valid)
        if self.noise is not None:
            luminance = self.noise.filter(luminance)
            # Bound in [0,1]
            luminance = np.maximum(0, luminance)
            luminance = np.minimum(1, luminance)
        luminance[invalid] = self.invalid
        readings[invalid] = self.invalid_range

#         delta_size = len(self.delta)
#         n = len(self.directions_o)
# 
#         def smooth(what):
#             res = np.zeros(n)
#             for i in range(n):
#                 other = what[i * delta_size:(i + 1) * delta_size]
#                 res[i] = np.sum(other * self.coeff)
#             return res
# 
#         valid2 = np.zeros(n, dtype=bool)
#         readings2 = np.zeros(n, dtype='float32')
# 
#         for i in range(n):
#             interval = np.array(range(i * delta_size, (i + 1) * delta_size))
#             if np.any(valid[interval]):
#                 valid2[i] = True
#                 subset = interval[valid[interval]]
# 
#                 # TODO: there is a bug with infinity, for the readings
#                 which = int((i + 0.5) * delta_size)
#                 if valid[which]:
#                     readings2[i] = readings[which]
#                 else:
#                     readings2[i] = np.mean(readings[subset])
#             else:
#                 valid2[i] = False

# #         luminance2 = smooth(data['luminance'])
# #         luminance2 = np.maximum(0, np.minimum(1, luminance2))
# #         luminance2[np.logical_not(valid2)] = self.invalid
# 
#         sensels = luminance2.copy()
#         # XXX: this step should not be necessary
#         sensels[np.isnan(sensels)] = self.invalid

        luminance2 = self.smoother.smooth(luminance)
        readings2 = self.smoother.smooth(readings)
        valid2 = np.array([True] * len(luminance2))

        data = {'luminance': luminance2,
                 
                'readings': readings2,
                'directions': self.directions_o.tolist(),
                'valid': valid2.tolist(),
                VehicleSensor.SENSELS: luminance2.copy(),
                
                # sampled extra rays
                'directions_raw': self.directions,
                'luminance_raw': luminance,
                }

        return data

    def set_world_primitives(self, primitives):
        TexturedRaytracer.set_world_primitives(self, primitives)


class PhotoreceptorsSmoothUniform(PhotoreceptorsSmooth):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, spatial_sigma_deg=10, upsample=7, noise=None):
        from vehicles.library.sensors.utils import get_uniform_directions
        directions = get_uniform_directions(fov_deg, num_sensels)
        PhotoreceptorsSmooth.__init__(self, directions=directions,
                                      spatial_sigma_deg=spatial_sigma_deg,
                                      upsample=upsample,
                                        noise=noise)


class PhotoreceptorsSmoothRandom(PhotoreceptorsSmooth):
    """ Photoreceptors with random disposition of sensels. """
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, spatial_sigma_deg=10,
                 upsample=7, noise=None):
        from vehicles.library.sensors.utils import get_random_directions
        directions = get_random_directions(fov_deg, num_sensels)
        PhotoreceptorsSmooth.__init__(self, directions=directions,
                                      spatial_sigma_deg=spatial_sigma_deg,
                                      upsample=upsample,
                                        noise=noise)

class PhotoreceptorsPerturb(PhotoreceptorsSmooth):
    """ Photoreceptors with random disposition of sensels. """
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, spatial_sigma_deg,
                 upsample, perturb_deg):
    
        from vehicles.library.sensors.utils import get_uniform_directions
        directions = get_uniform_directions(fov_deg, num_sensels)
#         perturb = np.deg2rad(perturb_deg)
#         directions_n = np.random.randn(len(directions)) * perturb
#         directions = directions + directions_n
        
        directions2 = get_uniform_directions(fov_deg, num_sensels * upsample)
        perturb = np.deg2rad(perturb_deg)
        directions2_n = np.random.randn(len(directions2)) * perturb
        
        directions2 = directions2 + directions2_n
        
        PhotoreceptorsSmooth.__init__(self, directions=directions,
                                      spatial_sigma_deg=spatial_sigma_deg,
                                      upsample=upsample,
                                      d2=directions2,
                                      noise=None)


