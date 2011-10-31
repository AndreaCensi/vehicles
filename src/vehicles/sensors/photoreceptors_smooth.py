from . import TexturedRaytracer, contract, np, get_uniform_directions
from ..interfaces import VehicleSensor
from conf_tools import instantiate_spec
from geometry import SE2_project_from_SE3

__all__ = ['PhotoreceptorsSmooth', 'PhotoreceptorsSmoothUniform']

class PhotoreceptorsSmooth(VehicleSensor, TexturedRaytracer):
    """ Implements  """
    
    @contract(directions='seq[>0](number)', spatial_sigma_deg='>0')
    def __init__(self, directions, spatial_sigma_deg,
                 noise=None, invalid=0.5):
        self.invalid = invalid
        
        self.noise_spec = noise    
        self.noise = (None if self.noise_spec is None else 
                          instantiate_spec(self.noise_spec))
            
        spec = {
            'desc': 'Photoreceptors',
            'shape': [len(directions)],
            'format': 'C',
            'range': [0, +1],
            'extra': {'directions': directions.tolist(),
                      'noise': self.noise_spec,
                      'spatial_sigma_deg': spatial_sigma_deg},
        }
        VehicleSensor.__init__(self, spec)
         
         
        def directions_from_angles(theta):
            return np.vstack((np.cos(theta), np.sin(theta)))
        
        def cosines_from_directions(S):
            C = np.dot(S.T, S)
            return np.clip(C, -1, 1, C)
        
        def distances_from_cosines(C):
            return np.real(np.arccos(C))

        def cosines_from_distances(D): 
            return np.cos(D)

        def distances_from_directions(S):
            C = cosines_from_directions(S)
            return distances_from_cosines(C)
                    
        S = directions_from_angles(directions)
        D = distances_from_directions(S)
        
        def kernel(x):
            a = np.deg2rad(spatial_sigma_deg)
            return np.exp(-(x / a) ** 2)
        self.coeff = kernel(D)
        
        assert np.all(np.isfinite(self.coeff))
        for i in range(len(directions)):
            sum_i = self.coeff[i, :].sum() 
            assert sum_i > 0
            self.coeff[i, :] = self.coeff[i, :] / sum_i

        assert np.all(np.isfinite(self.coeff))
#        import scipy.stats
#        eps = 0.1
#        resolution = 7
#        offsets = scipy.stats.norm.ppf(np.linspace(eps, 1 - eps, resolution)) 
#        rawdirs = []
#         
            
        TexturedRaytracer.__init__(self, directions)


    def to_yaml(self):
        return {'type': 'Photoreceptors',
                'noise': self.noise_spec,
                'invalid': self.invalid,
                'directions': self.directions.tolist()}

    def _compute_observations(self, pose):
        pose = SE2_project_from_SE3(pose)
        data = self.raytracing(pose)
        luminance = data['luminance']
        invalid = np.logical_not(data['valid'])
        if self.noise is not None:
            luminance = self.noise.filter(luminance)
            # Bound in [0,1]
            luminance = np.maximum(0, luminance)
            luminance = np.minimum(1, luminance)
        luminance[invalid] = self.invalid
        luminance_smooth = np.dot(self.coeff, luminance)
        data[VehicleSensor.SENSELS] = luminance_smooth
        return data

    def set_world_primitives(self, primitives):
        TexturedRaytracer.set_world_primitives(self, primitives)


class PhotoreceptorsSmoothUniform(PhotoreceptorsSmooth):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, spatial_sigma_deg=0, noise=None):
        directions = get_uniform_directions(fov_deg, num_sensels)
        PhotoreceptorsSmooth.__init__(self, directions=directions,
                                      spatial_sigma_deg=spatial_sigma_deg,
                                        noise=noise)
        
