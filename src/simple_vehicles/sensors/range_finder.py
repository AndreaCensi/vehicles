from simple_vehicles.interfaces import VehicleSensor
from simple_vehicles.sensors import Raytracer, TexturedRaytracer
from contracts import contract
import numpy as np

class Rangefinder(VehicleSensor, Raytracer):

    @contract(directions='seq[>0](number)')
    def __init__(self, directions):
        VehicleSensor.__init__(self, len(directions))
        Raytracer.__init__(self, directions)
    
    def _compute_observations(self, pose):
        data = self.raytracing(pose)
        # TODO: nans?
        data[VehicleSensor.SENSELS] = data['readings']
        return data

    def set_world(self, world, updated):
        Raytracer.set_world(self, world, updated)
    
class Photoreceptors(VehicleSensor, TexturedRaytracer):
    """ This is a very shallow wrap around ImageRangeSensor """
    @contract(directions='seq[>0](number)')
    def __init__(self, directions):
        VehicleSensor.__init__(self, len(directions))
        TexturedRaytracer.__init__(self, directions)
        
    def _compute_observations(self, sensor_pose):
        data = self.raytracing(sensor_pose)
        data[VehicleSensor.SENSELS] = data['luminance']
        return data

    def set_world(self, world, updated):
        TexturedRaytracer.set_world(self, world, updated)
    
    
class RangefinderUniform(Rangefinder):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels):
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
        Rangefinder.__init__(self, directions)
        
class PhotoreceptorsUniform(Photoreceptors):
    @contract(fov_deg='>0,<=360', num_sensels='int,>0')
    def __init__(self, fov_deg, num_sensels):
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)
        Photoreceptors.__init__(self, directions)
        

        
