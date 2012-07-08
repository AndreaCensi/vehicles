from . import FieldSampler, contract, np
from vehicles.library.sensors.utils import get_uniform_directions

__all__ = ['FieldSamplerCircle']


class FieldSamplerCircle(FieldSampler):
    ''' 
        A field sampler with sensels distributed in a circle 
        of a certain radius. 
    '''

    @contract(radius='>0', n='int,>0', fov_deg='>0')
    def __init__(self, n, radius, fov_deg, **params):
        self.radius = radius
        self.fov_deg = fov_deg

        p = get_uniform_directions(fov_deg, n)
        positions = map(lambda theta: [radius * np.cos(theta),
                                       radius * np.sin(theta)], p)
        FieldSampler.__init__(self, positions=positions, **params)

    def to_yaml(self):
        s = FieldSampler.to_yaml(self)
        s['radius'] = self.radius
        s['fov_deg'] = self.fov_deg
        return s
