import numpy as np
from contracts import contract


class RandomCheckerboard:
    @contract(num='int,>=1', cell_width='>0')
    def __init__(self, cell_width=1, num=100, seed=0):
        generator = np.random.mtrand.RandomState(seed)
        self.values = generator.uniform(0, 1, num)
        self.cell_width = cell_width
        
    def __call__(self, t):
        s = int(np.floor(t / self.cell_width))
        s = s % self.values.size
        return self.values[s]
        
