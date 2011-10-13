from . import np, contract
from ..interfaces import Field, VehicleSensor
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3


class FieldSampler(VehicleSensor):
    ''' A sensor that samples an intensity field. '''
        
    @contract(positions='seq[>0](seq[2](number))')
    def __init__(self, positions, min_value=0, max_value=1, noise=None):
        ''' 
            :param positions: 2D positions of the sensels 
        '''
        self.num_sensels = len(positions)
        self.positions = np.array(positions)
        self.min_value = min_value
        self.max_value = max_value
        
        self.noise_spec = noise
        self.noise = (None if self.noise_spec is None else 
                          instantiate_spec(self.noise_spec))
        
        boot_spec = {
            'desc': 'Field sampler',
            'shape': [len(positions)],
            'format': 'C',
            'range': [self.min_value, self.max_value],
            'extra': {'positions': positions,
                      'noise_spec': self.noise_spec }
        }
        VehicleSensor.__init__(self, boot_spec)
        
        self.primitives = None
    
    def to_yaml(self):
        return {'type': 'FieldSampler',
                'noise_spec': self.noise_spec,
                'min_value': self.min_value,
                'max_value': self.max_value,
                'positions': self.positions.tolist()}
    
    @contract(pose='SE3')
    def _compute_observations(self, pose):
        if self.primitives is None:
            raise ValueError('Primitives not set yet.')
        pose = SE2_project_from_SE3(pose)
        sensels = np.zeros(self.num_sensels)
        for i in range(self.num_sensels):
            upoint = np.hstack((self.positions[i], 1))
            world_point = np.dot(pose, upoint)[:2] # XXX
            sensels[i] = get_field_value(self.primitives, world_point)
        
        if self.noise is not None:
            sensels = self.noise.filter(sensels)
        
        sensels = np.minimum(self.max_value, sensels)
        sensels = np.maximum(self.min_value, sensels)
            
        data = dict(sensels=sensels)
        return data

    def set_world_primitives(self, primitives):
        self.primitives = primitives

def get_field_value(primitives, point):
    values = []
    for p in primitives:
        if isinstance(p, Field):
            intensity = p.get_intensity_value(point)
            values.append(intensity)
    if len(values) == 0:
        return 0
    else:
        return np.mean(values)

def get_field_values(primitives, X, Y):
    values = []
    for p in primitives:
        if isinstance(p, Field):
            intensity = p.get_intensity_values(X, Y)
            values.append(intensity)
    if len(values) == 0:
        return 0
    else:
        return np.mean(values, axis=0)

   
