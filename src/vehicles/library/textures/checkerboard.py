from .sampled import SampledTexture
from contracts import contract
import numpy as np


__all__ = [
    'RandomCheckerboard', 
    'BWCheckerboard',
    'OneEdge',
]


class RandomCheckerboard(SampledTexture):

    @contract(num='int,>=1', cell_width='>0')
    def __init__(self, cell_width=1, num=100, seed=0):
        generator = np.random.mtrand.RandomState(seed)
        values = generator.uniform(0, 1, num)

        SampledTexture.__init__(self, values, cell_width)


class BWCheckerboard(SampledTexture):

    @contract(num='int,>=1', cell_width='>0')
    def __init__(self, cell_width=1, num=100):
        values = np.zeros(num)
        for i in range(num):
            values[i] = i % 2
        SampledTexture.__init__(self, values, cell_width)


class OneEdge(SampledTexture):
    def __init__(self, v0=0.001, v1=1.000, where=10, hardness=1.0):
        self.v0 = float(v0)
        self.v1 = float(v1)
        self.hardness = float(hardness)
        self.where = float(where)
        
    def __call__(self, t):
        t = t.copy() # important
        radius = 10.0
        t[t > radius * np.pi] -= 2 * np.pi * radius
         
        x = t - self.where
        x = x * self.hardness
        y = sigmoid(x)
        return y
         
def sigmoid(x):                                        
    return 1.0 / (1.0 + np.exp(-x))

