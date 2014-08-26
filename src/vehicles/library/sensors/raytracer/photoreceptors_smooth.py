from contracts import contract

from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3
import numpy as np
from vehicles import VehicleSensor, VehiclesConstants

from .textured_raytracer import TexturedRaytracer


__all__ = [
    'PhotoreceptorsSmooth',
    'PhotoreceptorsSmoothUniform',
    'PhotoreceptorsSmoothRandom',
]

class Smoother():
    def __init__(self, directions, spatial_sigma_deg, upsample):
        self.directions = directions
        spatial_sigma_rad = np.deg2rad(spatial_sigma_deg)
        
        self.delta_norm =np.linspace(-2, 2, upsample) 
        self.delta = self.delta_norm  * spatial_sigma_rad
        
        def kernel(x):
            return np.exp(-(x ** 2)/2) / np.sqrt(2*np.pi)

        self.coeff = kernel(self.delta_norm)
        self.coeff = self.coeff / np.sum(self.coeff)
        
        assert len(self.coeff) == upsample
        assert len(self.delta) == len(self.coeff)
        self.M = np.zeros((len(directions), upsample * len(directions)))
        
        i = 0
        self.directions2 = []
        for k, d in enumerate(directions):
            for r, s in enumerate(self.delta):
                self.directions2.append(d + s)
                self.coeff[r]
                self.M[k, i] = self.coeff[r]
                i += 1
 
 
    def get_new_directions(self):
        return self.directions2
    
    def smooth(self, sampled):
        assert np.all(np.isfinite(sampled))
        sampled = np.array(sampled)
        
        if not len(sampled) == len(self.directions2):
            msg = 'invalid shape: %s vs %s' % (sampled.shape, len(self.directions2))
            raise ValueError(msg)

        return np.dot(self.M, sampled)
    
 

class PhotoreceptorsSmooth(VehicleSensor, TexturedRaytracer):
    """ Implements  """

    @contract(directions='seq[>0](number)', spatial_sigma_deg='>=0')
    def __init__(self, directions, spatial_sigma_deg, upsample=7,
                 noise=None, invalid=0.5):
        if upsample< 5:
            print('warning: upsample %d is small' % upsample)
        
        self.invalid = invalid
        self.invalid_range = 100 # xxx

        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else
                          instantiate_spec(self.noise_spec))

        #d = np.array(directions)
        #print d[:-1] - d[1:]
        self.smoother = Smoother(directions=directions, 
                                 spatial_sigma_deg=spatial_sigma_deg, 
                                 upsample=upsample)
                                 
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
                VehicleSensor.SENSELS: luminance2.copy()}

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
