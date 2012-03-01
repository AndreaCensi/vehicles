from . import np, contract, Texture

# In all of these, t can be either a scalar or a numpy array.


class SinTexture(Texture):

    @contract(c='number', A='number', omega='number')
    def __init__(self, omega, c=0.5, A=0.5):
        self.omega = omega
        self.c = c
        self.A = A

    def __call__(self, t):
        return self.c + self.A * np.sin(self.omega * t)
