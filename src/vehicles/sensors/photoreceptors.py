from . import TexturedRaytracer, contract, np
from ..interfaces import VehicleSensor
from geometry.poses_embedding import SE2_project_from_SE3

class Photoreceptors(VehicleSensor, TexturedRaytracer):
    """ This is a very shallow wrap around ImageRangeSensor """
    @contract(directions='seq[>0](number)')
    def __init__(self, directions, invalid=0.5):
        self.invalid = invalid
        VehicleSensor.__init__(self, len(directions))
        TexturedRaytracer.__init__(self, directions)
        
    def _compute_observations(self, pose):
        pose = SE2_project_from_SE3(pose)
        data = self.raytracing(pose)
        luminance = data['luminance']
        invalid = np.logical_not(data['valid'])
        luminance[invalid] = self.invalid
        data[VehicleSensor.SENSELS] = luminance
        return data

    def set_world_primitives(self, primitives):
        TexturedRaytracer.set_world_primitives(self, primitives)


class PhotoreceptorsUniform(Photoreceptors):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels):
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
        Photoreceptors.__init__(self, directions)
        
