from . import random_checkerboard, box
from ..interfaces import Circle, World
from contracts import contract
import numpy as np
from vehicles.interfaces.primitives import Source

class StochasticBox2(World):
    ''' This also includes point sources. '''
    
    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10,
                 num_circles=15, circles_size=[1, 3],
                 num_sources=10, source_width=3):
        self.width = width
        self.length = length
        self.circles_size = circles_size
        r = 1
        bounds = [[-width * r, +width * r],
                  [-length * r, +length * r],
                  [0, 5]]
        World.__init__(self, bounds)
        self.bounds = bounds

        self.box = box(0, random_checkerboard(0.5), width, length)
       
        self.circles = []
        id_object = 0 
        for _ in range(num_circles):
            c = Circle(id_object=id_object, tags=[],
                     texture=random_checkerboard(0.1),
                     center=[0, 0],
                     radius=1,
                     solid=True)
            self.circles.append(c)
            id_object += 1
            
        self.sources = []
        for _ in range(num_sources):
            c = Source(id_object=id_object, tags=[],
                     center=[0, 0],
                     kernel_spec=KernelExponential.spec(source_width))
            self.sources.append(c)
            id_object += 1

            
        self.refresh()
        
    def refresh(self):
        for c in self.circles:
            c.set_center(self.get_random_point())
            c.radius = np.random.uniform(self.circles_size[0],
                                         self.circles_size[1])
        for s in self.sources:
            s.set_center(self.get_random_point())
            
    def get_random_point(self):
        x = np.random.uniform(self.bounds[0][0], self.bounds[0][1])
        y = np.random.uniform(self.bounds[1][0], self.bounds[1][1])
        return [x, y]
    
    def get_primitives(self):
        return [self.box] + self.circles + self.sources
    
    def new_episode(self):
        self.refresh()
        return World.new_episode(self)

    def simulate(self, dt, vehicle_pose):
        return [] # no changes

    
    
class KernelExponential():
    def __init__(self, L):
        ''' L is the size of 99% area '''
        self.L = L
        
    def __call__(self, d):
        d = np.array(d)
        return np.exp(-d / self.L)
        
    @staticmethod
    def spec(L):
        return ['vehicles.worlds.KernelExponential', {'L': L}]
    
