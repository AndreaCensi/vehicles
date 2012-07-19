from . import np, contract, logger
from vehicles import Field, VehicleSensor, Source, VehiclesConstants
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3

__all__ = ['FieldSampler', 'get_field_values', 'get_field_value']


class FieldSampler(VehicleSensor):
    ''' A sensor that samples an intensity field. '''

    @contract(positions='seq[>0](seq[2](number))')
    def __init__(self, positions,
                 min_value=0, max_value=1, normalize=False, noise=None,
                 shape=None):
        '''
            :param positions: 2D positions of the sensels 
        '''

        if normalize:
            logger.warning('normalize=True is deprecated')
            
        if shape is None:
            shape = [len(positions)]

        self.num_sensels = len(positions)
        self.positions = np.array(positions)
        self.min_value = min_value
        self.max_value = max_value
        self.normalize = normalize

        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else
                          instantiate_spec(self.noise_spec))

        boot_spec = {
            'desc': 'Field sampler',
            'shape': shape,
            'format': 'C',
            'range': [float(self.min_value), float(self.max_value)],
            'extra': {'positions': self.positions.tolist(),
                      'normalize': bool(normalize),
                      'max_value': float(max_value),
                      'min_value': float(min_value),
                      'noise': self.noise_spec}
        }

        VehicleSensor.__init__(self, boot_spec)

        self.primitives = None

    def to_yaml(self):
        return {'type': VehiclesConstants.SENSOR_TYPE_FIELDSAMPLER,
                'noise_spec': self.noise_spec,
                'min_value': self.min_value,
                'max_value': self.max_value,
                'positions': self.positions.tolist(),
                'normalize': self.normalize}

    @contract(pose='SE3')
    def _compute_observations(self, pose):
        if self.primitives is None:
            raise ValueError('Primitives not set yet.')
        pose = SE2_project_from_SE3(pose)
        
        # x,y coordinates of the field
        X = np.zeros(self.num_sensels)
        Y = np.zeros(self.num_sensels)
        for i in range(self.num_sensels):
            # Translate and rotate
            upoint = np.hstack((self.positions[i], 1))
            world_point = np.dot(pose, upoint)[:2]  # XXX
            X[i] = world_point[0]
            Y[i] = world_point[1]

        sensels = get_field_values(self.primitives, X, Y)

        # TODO: I think this can be removed safely
        if self.normalize:
            sensels -= np.min(sensels)
            if np.max(sensels) > 0:
                sensels *= self.max_value / np.max(sensels)

        if self.noise is not None:
            sensels = self.noise.filter(sensels)

        sensels = np.clip(sensels, self.min_value, self.max_value)

        data = dict(sensels=sensels)
        return data

    def set_world_primitives(self, primitives):
        # FIXME this does not work with changing environments
        if primitives:  # FIXME: only find changed things
            sources = [p for p in primitives if isinstance(p, Source)]
            if not sources:
                logger.error('Warning: there no sources given for a field '
                             'sampler in this world.')
        if self.primitives is None:
            self.primitives = primitives


def get_field_value(primitives, point):
    values = []
    for p in primitives:
        if isinstance(p, Source):
            intensity = p.get_intensity_value(point)
            values.append(intensity)
    if len(values) == 0:
        return 0
    else:
        return np.mean(values) # XXX: why?


@contract(X='shape(x)', Y='shape(x)', returns='array,shape(x)')
def get_field_values(primitives, X, Y):
    values = []
    for p in primitives:
        if isinstance(p, Field):
            intensity = p.get_intensity_values(X, Y)
            values.append(intensity)
    if len(values) == 0:
        return 0
    else:
        return np.mean(values, axis=0) # XXX: why?


