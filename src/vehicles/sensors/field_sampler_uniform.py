from . import FieldSampler, contract, np
import itertools

__all__ = ['FieldSamplerUniform']


class FieldSamplerUniform(FieldSampler):
    ''' A field sampler with uniform disposition of sensels. '''

    @contract(shape='seq[2](number,>1)', sides='seq[2](number)')
    def __init__(self, shape, sides, normalize=False, min_value=0, max_value=1,
                 noise=None):
        ''' 
            :param sides: sides of the rectangle comprising the sensels
            :param shape: 2D shape of the sensels 
            :param noise_spec: code spec for the noise added 
        '''
        self.shape = list(shape)
        self.sides = list(sides)
        self.cell2index = np.zeros(self.shape, 'int')
        positions = []
        k = 0
        for i, j in itertools.product(range(shape[0]),
                                      range(shape[1])):

            u = 2 * (i - shape[0] / 2.0) / shape[0]
            v = 2 * (j - shape[1] / 2.0) / shape[1]
            x = -u * self.sides[0]
            y = -v * self.sides[1]
            positions.append([x, y])
            self.cell2index[i, j] = k
            k += 1

        FieldSampler.__init__(self,
                              shape=shape,
                              positions=positions,
                              min_value=min_value,
                              max_value=max_value,
                              normalize=normalize,
                              noise=noise)

    @contract(pose='SE3')
    def _compute_observations(self, pose):
        # FIXME: this will not be drawn correctly
        sensels = FieldSampler._compute_observations(self, pose)['sensels']
        sensels = sensels[self.cell2index]
        return dict(sensels=sensels)

    def to_yaml(self):
        s = FieldSampler.to_yaml(self)
        s['shape'] = self.shape
        s['sides'] = self.sides
        s['cell2index'] = self.cell2index.tolist()
        return s



