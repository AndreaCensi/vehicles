from . import np, contract

__all__ = ['AdditiveGaussian']


class AdditiveGaussian:
    ''' Implements additive Gaussian noise with fixed variance. '''
    def __init__(self, std_dev):
        self.std_dev = std_dev

    @contract(values='array[K]', returns='array[K]')
    def filter(self, values):  # @ReservedAssignment
        return values + self.std_dev * np.random.randn(values.size)
