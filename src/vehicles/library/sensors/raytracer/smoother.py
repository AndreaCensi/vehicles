import numpy as np

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
        
         
        print('delta_norm: %s rad' % self.delta_norm)
        print('delta : %s deg' % np.rad2deg( self.delta))
        
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
# 
#         self.coeff = kernel(self.delta_norm)
#         self.coeff = self.coeff / np.sum(self.coeff)
#         
#         assert len(self.coeff) == upsample
#         assert len(self.delta) == len(self.coeff)
        
        
        i = 0
        # for each direction
        self.directions2 = []
        for d in directions:
            for s in self.delta:
                self.directions2.append(d + s)
        
        self.M = np.zeros((len(directions), len(self.directions2)))
         
        for i, di in enumerate(directions):
            for j, dj in enumerate(self.directions2):
                diff = dj-di
                coeff = kernel(diff / spatial_sigma_rad)
                self.M[i, j] = coeff
 
        for i in range(len(directions)):
            self.M[i, :] = self.M[i, :] / np.sum(self.M[i,:])
            
    def get_new_directions(self):
        return self.directions2
    
    def smooth(self, sampled):
        assert np.all(np.isfinite(sampled))
        sampled = np.array(sampled)
        
        if not len(sampled) == len(self.directions2):
            msg = 'invalid shape: %s vs %s' % (sampled.shape, len(self.directions2))
            raise ValueError(msg)

        return np.dot(self.M, sampled)