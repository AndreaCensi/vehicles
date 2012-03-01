from abc import abstractmethod, ABCMeta


class Texture:
    ''' 
        A texture is simply a function from R to R.
        
        Note that the implementation must be able to accept 
        numpy arrays and do vectorized evaluation.
    '''

    __metaclass__ = ABCMeta

    @abstractmethod
    def __call__(self, x):
        ''' x can be a number or a 1D numpy array. '''
