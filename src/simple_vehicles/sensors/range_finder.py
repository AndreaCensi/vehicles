from simple_vehicles.interfaces import VehicleSensor
from simple_vehicles.sensors import Raytracer, TexturedRaytracer
from contracts import contract
import numpy as np

class Rangefinder(VehicleSensor, Raytracer):

    @contract(directions='seq[>0](number)')
    def __init__(self, directions):
        VehicleSensor.__init__(self, len(directions))
        Raytracer.__init__(self, directions)
    
    def compute_observations(self, pose, dt):
        data = self.raytracing(pose)
        data['sensels'] = data['readings']
        return data


class Nearnessfinder(VehicleSensor, Raytracer):
    """ Same as Rangefinder, but we return the nearness 
        instead of ranges as sensels. """

    @contract(directions='seq[>0](number)')
    def __init__(self, directions):
        VehicleSensor.__init__(self, len(directions))
        Raytracer.__init__(self, directions)
    
    def compute_observations(self, pose, dt):
        data = self.raytracing(pose)
        data['sensels'] = 1.0 / data['readings']
        return data

class Photoreceptors(VehicleSensor, TexturedRaytracer):
    """ This is a very shallow wrap around ImageRangeSensor """
    @contract(directions='seq[>0](number)')
    def __init__(self, directions):
        VehicleSensor.__init__(self, len(directions))
        Raytracer.__init__(self, directions)
        
    def compute_observations(self, sensor_pose):
        data = self.render(sensor_pose)
        data['sensels'] = data['luminance']
        return data
    
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
        

        
