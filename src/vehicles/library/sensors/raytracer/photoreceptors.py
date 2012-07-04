from . import TexturedRaytracer, contract, np
from .. import get_uniform_directions
from vehicles import VehicleSensor, VehiclesConstants
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3

__all__ = ['Photoreceptors', 'PhotoreceptorsUniform']


class Photoreceptors(VehicleSensor, TexturedRaytracer):
    """ This is a very shallow wrapper around ImageRangeSensor """

    @contract(directions='seq[>0](number)')
    def __init__(self, directions, noise=None, invalid=0.5):
        self.invalid = invalid

        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else
                          instantiate_spec(self.noise_spec))

        spec = {
            'desc': 'Photoreceptors',
            'shape': [len(directions)],
            'format': 'C',
            'range': [0, +1],
            'extra': {'directions': directions.tolist(),
                      'invalid': invalid,
                      'sono': 'qui',
                      'noise': self.noise_spec}
        }
        VehicleSensor.__init__(self, spec)

        TexturedRaytracer.__init__(self, directions)

    def to_yaml(self):
        return {'type': VehiclesConstants.SENSOR_TYPE_PHOTORECEPTORS,
                'noise': self.noise_spec,
                'invalid': self.invalid,
                'directions': self.directions.tolist()}

    def _compute_observations(self, pose):
        pose = SE2_project_from_SE3(pose)
        data = self.raytracing(pose)
        luminance = data['luminance']
        invalid = np.logical_not(data['valid'])
        if self.noise is not None:
            luminance = self.noise.filter(luminance)
            # Bound in [0,1]
            luminance = np.maximum(0, luminance)
            luminance = np.minimum(1, luminance)
        luminance[invalid] = self.invalid
        data[VehicleSensor.SENSELS] = luminance
        return data

    def set_world_primitives(self, primitives):
        TexturedRaytracer.set_world_primitives(self, primitives)


class PhotoreceptorsUniform(Photoreceptors):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, noise=None):
        directions = get_uniform_directions(fov_deg, num_sensels)
        Photoreceptors.__init__(self, directions=directions,
                                        noise=noise)

