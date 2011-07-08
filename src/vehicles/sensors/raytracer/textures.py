import numpy as np
from contracts import contract

class ConstantTexture:
    def __init__(self, value):
        self.value = value
    def __call__(self, t):
        return self.value

class SinTexture:
    
    @contract(c='number', A='number', omega='number')
    def __init__(self, omega, c=0.5, A=0.5):
        self.omega = omega
        self.c = c
        self.A = A
        
    def __call__(self, t):
        return self.c + self.A * np.sin(self.omega * t)
