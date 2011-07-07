
from ..interfaces import World, PolyLine
import numpy as np
from contracts.main import contract
from simple_vehicles.interfaces.world_interface import Circle
from geometry.poses import SE2_from_xytheta


class Box(World):
    ''' A simple box. '''
    
    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10):
        self.width = width
        self.length = length
        texture = ['simple_vehicles.sensors.ConstantTexture', {'value': 0.5}]
        points = [ [-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
        
        points = [ (np.array(p) * np.array([width, length])).tolist() for p in points]
        self.box = PolyLine(id_object=0, tags=[], texture=texture, points=points)

    def get_primitives(self):
        return [self.box]
    
    def _simulate(self, dt):
        return []
    
    def new_episode(self):
        x = np.random.uniform(-self.width, self.width)
        y = np.random.uniform(-self.length, self.length)
        th = np.random.uniform(0, np.pi * 2)
        vehicle_state = SE2_from_xytheta([x, y, th])
        id_episode = 'unknown'
        return World.Episode(id_episode, vehicle_state)

class BoxAndCircle(World):
    
    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10):
        texture = ['simple_vehicles.sensors.ConstantTexture', {'value': 0.5}]
        points = [ [-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
        
        points = [ (np.array(p) * np.array([width, length])).tolist() for p in points]
        self.box = PolyLine(id_object=0, tags=[], texture=texture, points=points)
        
        texture = ['simple_vehicles.sensors.ConstantTexture', {'value': 0.8}]
        self.circle = Circle(id_object=1, tags=[],
                             texture=texture,
                             center=[-width, -length],
                             radius=0.2 * width,
                             solid=True)
    
    def get_primitives(self):
        return [self.box, self.circle]
    
    def _simulate(self, dt):
        return []

    def new_episode(self):
        x = np.random.uniform(-self.width, self.width)
        y = np.random.uniform(-self.length, self.length)
        th = np.random.uniform(0, np.pi * 2)
        vehicle_state = SE2_from_xytheta([x, y, th])
        id_episode = 'unknown'
        return World.Episode(id_episode, vehicle_state)
