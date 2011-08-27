from . import TexturedRaytracer, contract, np
from ..configuration import instantiate_spec
from ..interfaces import VehicleSensor
from geometry import SE2_project_from_SE3

class Photoreceptors(VehicleSensor, TexturedRaytracer):
    """ This is a very shallow wrap around ImageRangeSensor """
    
    @contract(directions='seq[>0](number)')
    def __init__(self, directions, noise=None, invalid=0.5):
        self.invalid = invalid
        VehicleSensor.__init__(self, len(directions))
        TexturedRaytracer.__init__(self, directions)
        
        self.noise_spec = noise
        
        if noise is None:
            self.noise = None
        else:
            self.noise = instantiate_spec(noise)

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
            
        luminance[invalid] = self.invalid
        data[VehicleSensor.SENSELS] = luminance
        return data

    def set_world_primitives(self, primitives):
        TexturedRaytracer.set_world_primitives(self, primitives)


class PhotoreceptorsUniform(Photoreceptors):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels, noise=None):
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
        Photoreceptors.__init__(self, directions=directions,
                                        noise=noise)
        
