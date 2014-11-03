import numpy as np
from geometry.spheres import normalize_pi

__all__ = ['Smoother', 'Smoother2']

class Smoother():
    def __init__(self, directions, spatial_sigma_deg, upsample):
        self.directions = directions
        spatial_sigma_rad = np.deg2rad(spatial_sigma_deg)
         
        if upsample == 1:
            self.delta_norm = np.array([0.0])
        else:
            self.delta_norm = np.linspace(-2.0, 2.0, upsample)
        
        self.delta = self.delta_norm  * spatial_sigma_rad
        
         
#         print('delta_norm: %s rad' % self.delta_norm)
#         print('delta : %s deg' % np.rad2deg( self.delta))
#         
        def kernel(x):
            return np.exp(-(x ** 2.0)/2.0) / np.sqrt(2*np.pi)

        self.coeff = kernel(self.delta_norm)
        self.coeff = self.coeff / np.sum(self.coeff)
        
        assert len(self.coeff) == upsample
        assert len(self.delta) == len(self.coeff)
        self.M = np.zeros((len(directions), upsample * len(directions)))
        
        i = 0
        self.directions2 = []
        for k, d in enumerate(directions):
            for r, s in enumerate(self.delta):
                self.directions2.append(d + s)
                self.M[k, i] = self.coeff[r]
                i += 1
 
 
    def get_new_directions(self):
        return self.directions2
    
    def smooth(self, sampled):
        assert np.all(np.isfinite(sampled))
        sampled = np.array(sampled)
        
        if not len(sampled) == len(self.directions2):
            msg = 'invalid shape: %s vs %s' % (sampled.shape, len(self.directions2))
            raise ValueError(msg)

        return np.dot(self.M, sampled)
    
    
class Smoother2():
    def __init__(self, directions, spatial_sigma_deg, upsample, d2=None):
        '''
        
        :param directions:
        :param spatial_sigma_deg:
        :param upsample:
        :param d2: The directions to sample (or None)
        '''
        self.directions = directions
       
        # print('delta_norm: %s rad' % self.delta_norm)
        # print('delta : %s deg' % np.rad2deg( self.delta))
        #  
        def kernel(x):
            return np.exp(-(x ** 2.0)/2.0) / np.sqrt(2*np.pi)
        
        spatial_sigma_rad = np.deg2rad(spatial_sigma_deg)
        
        # for each direction
        if d2 is None:
            self.directions2 = self._get_directions_uniform(directions, 
                                        spatial_sigma_deg, upsample)
        else:
            self.directions2 = d2

        self.M = np.zeros((len(directions), len(self.directions2)))
        
        def normalize_pi(x):
            return np.arctan2(np.sin(x), np.cos(x))
         
        for i, di in enumerate(directions):
            for j, dj in enumerate(self.directions2):
                diff = dj-di # XXX: mod2
                diff = normalize_pi([diff])
                coeff = kernel(diff / spatial_sigma_rad)
                self.M[i, j] = coeff
 
        for i in range(len(directions)):
            self.M[i, :] = self.M[i, :] / np.sum(self.M[i,:])
            
        
    def _get_directions_uniform(self, directions, spatial_sigma_deg, upsample):
        spatial_sigma_rad = np.deg2rad(spatial_sigma_deg)
          
        if upsample == 1:
            self.delta_norm = np.array([0.0])
        else:
            self.delta_norm = np.linspace(-2.0, 2.0, upsample)
         
        self.delta = self.delta_norm  * spatial_sigma_rad

        directions2 = []
        for d in directions:
            for s in self.delta:
                directions2.append(d + s)
        return directions2
            
    def get_new_directions(self):
        return self.directions2
    
    def smooth(self, sampled):
        assert np.all(np.isfinite(sampled))
        sampled = np.array(sampled)
        
        if not len(sampled) == len(self.directions2):
            msg = 'invalid shape: %s vs %s' % (sampled.shape, len(self.directions2))
            raise ValueError(msg)

        return np.dot(self.M, sampled)