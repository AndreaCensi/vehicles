from . import TexturedRaytracer, contract, np
from .. import get_random_directions, get_uniform_directions
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3
from vehicles import VehicleSensor, VehiclesConstants

__all__ = ['PhotoreceptorsSmooth', 'PhotoreceptorsSmoothUniform',
           'PhotoreceptorsSmoothRandom']


class PhotoreceptorsSmooth(VehicleSensor, TexturedRaytracer):
    """ Implements  """

    @contract(directions='seq[>0](number)', spatial_sigma_deg='>0')
    def __init__(self, directions, spatial_sigma_deg, upsample=10,
                 noise=None, invalid=0.5):
        self.invalid = invalid

        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else
                          instantiate_spec(self.noise_spec))

        spatial_sigma_rad = np.deg2rad(spatial_sigma_deg)
        self.delta = np.linspace(-3, 3, upsample) * spatial_sigma_rad

        directions2 = []
        for d in directions:
            for s in self.delta:
                directions2.append(d + s)

        self.directions_o = np.array(directions)

        def kernel(x):
            return np.exp(-(x ** 2))

        self.coeff = kernel(self.delta)
        self.coeff = self.coeff / np.sum(self.coeff)

        spec = {
            'desc': 'Photoreceptors',
            'shape': [len(directions)],
            'format': 'C',
            'range': [0, +1],
            'extra': {'directions': self.directions_o.tolist(),
                      'noise': self.noise_spec,
                      'delta': self.delta.tolist(),
                      'coeff': self.coeff.tolist(),
                      'spatial_sigma_deg': spatial_sigma_deg},
        }

        VehicleSensor.__init__(self, spec)
        TexturedRaytracer.__init__(self, np.array(directions2))

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

        delta_size = len(self.delta)
        n = len(self.directions_o)

        def smooth(what):
            res = np.zeros(n)
            for i in range(n):
                other = what[i * delta_size:(i + 1) * delta_size]
                res[i] = np.sum(other * self.coeff)
            return res

        valid2 = np.zeros(n, dtype=bool)
        readings2 = np.zeros(n, dtype='float32')

        for i in range(n):
            interval = np.array(range(i * delta_size, (i + 1) * delta_size))
            if np.any(valid[interval]):
                valid2[i] = True
                subset = interval[valid[interval]]

                # TODO: there is a bug with infinity, for the readings
                which = int((i + 0.5) * delta_size)
                if valid[which]:
                    readings2[i] = readings[which]
                else:
                    readings2[i] = np.mean(readings[subset])
            else:
                valid2[i] = False

        luminance2 = smooth(data['luminance'])
        luminance2 = np.maximum(0, np.minimum(1, luminance2))
        luminance2[np.logical_not(valid2)] = self.invalid

        sensels = luminance2.copy()
        # XXX: this step should not be necessary
        sensels[np.isnan(sensels)] = self.invalid

        data = {'luminance': luminance2,
                'readings': readings2,
                'directions': self.directions_o.tolist(),
                'valid': valid2,
                VehicleSensor.SENSELS: sensels}

        return data

    def set_world_primitives(self, primitives):
        TexturedRaytracer.set_world_primitives(self, primitives)


class PhotoreceptorsSmoothUniform(PhotoreceptorsSmooth):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, spatial_sigma_deg=10,
                 upsample=5, noise=None):
        directions = get_uniform_directions(fov_deg, num_sensels)
        PhotoreceptorsSmooth.__init__(self, directions=directions,
                                      spatial_sigma_deg=spatial_sigma_deg,
                                      upsample=upsample,
                                        noise=noise)


class PhotoreceptorsSmoothRandom(PhotoreceptorsSmooth):
    """ Photoreceptors with random disposition of sensels. """
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, spatial_sigma_deg=10,
                 upsample=5, noise=None):
        directions = get_random_directions(fov_deg, num_sensels)
        PhotoreceptorsSmooth.__init__(self, directions=directions,
                                      spatial_sigma_deg=spatial_sigma_deg,
                                      upsample=upsample,
                                        noise=noise)
