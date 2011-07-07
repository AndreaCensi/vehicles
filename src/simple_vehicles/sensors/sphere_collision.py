from contracts import contract
from geometry import translation_angle_from_SE2
from ..interfaces import VehicleSensor
from . import Raytracer

class SphereCollision(VehicleSensor, Raytracer):

    @contract(radius='>0')
    def __init__(self, radius):
        VehicleSensor.__init__(self, 1)
        Raytracer.__init__(self, directions=None)
    
    def compute_observations(self, pose, dt):
        center, angle = translation_angle_from_SE2(pose)
        intersects, surface = self.query_circle(center, self.radius)
        data = dict(sensels=np.array([intersects]))
        return data
