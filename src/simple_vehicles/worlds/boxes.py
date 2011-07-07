
from ..interfaces import World, PolyLine
import numpy as np
from contracts.main import contract
from simple_vehicles.interfaces.world_interface import Circle


class Box(World):
    ''' A simple box. '''
    
    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10):
        texture = ['simple_vehicles.sensors.ConstantTexture', {'value': 0.5}]
        points = [ [-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
        
        points = [ (np.array(p) * np.array([width, length])).tolist() for p in points]
        self.box = PolyLine(id_object=0, tags=[], texture=texture, points=points)

    def get_primitives(self):
        return [self.box]


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
