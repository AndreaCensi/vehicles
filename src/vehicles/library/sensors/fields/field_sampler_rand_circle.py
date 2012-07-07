from . import FieldSampler, contract, np

__all__ = ['FieldSamplerRandCircle']


class FieldSamplerRandCircle(FieldSampler):
    ''' A field sampler with sensels randomly 
        distributed in a certain radius. '''

    @contract(radius='>0', n='int,>0')
    def __init__(self, n, radius, normalize=False, min_value=0, max_value=1,
                 noise=None):
        self.radius = radius

        positions = []
        for _ in range(n):
            p = random_in_circle(radius)
            positions.append(p)

        FieldSampler.__init__(self,
                              positions=positions,
                              min_value=min_value,
                              max_value=max_value,
                              normalize=normalize,
                              noise=noise)

    def to_yaml(self):
        s = FieldSampler.to_yaml(self)
        s['radius'] = self.radius
        return s


def random_in_circle(radius):
    ''' Random sample using rejection sampling. '''
    while True:
        p = np.random.uniform(-1, +1, 2)
        if np.linalg.norm(p) <= 1:
            return p * radius





