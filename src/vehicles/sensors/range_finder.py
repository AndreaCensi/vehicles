from . import Raytracer, contract, np, get_uniform_directions
from .. import VehicleSensor, VehiclesConstants
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3

__all__ = ['Rangefinder', 'RangefinderUniform']


class Rangefinder(VehicleSensor, Raytracer):

    @contract(directions='seq[>0](number)', invalid='number',
              min_range='>=0,x', max_range='>=0,>x')
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
            'extra': {'directions': list(directions),
                      'noise': noise}
        }
        VehicleSensor.__init__(self, spec)
        Raytracer.__init__(self, directions)

    def to_yaml(self):
        return {'type': VehiclesConstants.SENSOR_TYPE_RANGEFINDER,
                'noise_spec': self.noise_spec,
                'invalid': self.invalid,
                'min_range': self.min_range,
                'max_range': self.max_range,
                'directions': list(self.directions)}

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
        # XXX What if invalid is not in min_range, max_range
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

