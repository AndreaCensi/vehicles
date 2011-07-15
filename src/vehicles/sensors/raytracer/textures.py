import numpy as np
from contracts import contract

# In all of these, t can be either a scalar or a numpy array.

class ConstantTexture:
    def __init__(self, value):
        self.value = value
        
    def __call__(self, t):
        return t * 0 + self.value

class SinTexture:
    
    @contract(c='number', A='number', omega='number')
    def __init__(self, omega, c=0.5, A=0.5):
        self.omega = omega
        self.c = c
        self.A = A
        
    def __call__(self, t):
        return self.c + self.A * np.sin(self.omega * t)


class RandomCheckerboard:
    def __init__(self, cell_width=1, num=100):
        self.values = np.random.rand(num)
        self.cell_width = cell_width
        
    def __call__(self, t):
        s = int(np.floor(t / self.cell_width))
        s = s % self.values.size
        return self.values[s]
        
