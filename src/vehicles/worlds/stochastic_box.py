from . import random_checkerboard, box, np, contract
from ..interfaces import Circle, World

__all__ = ['StochasticBox']

class StochasticBox(World):
    
    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10, num_circles=15, circles_size=[1, 3]):
        self.width = width
        self.length = length
        self.num_circles = num_circles
        self.circles_size = circles_size
        r = 1
        bounds = [[-width * r, +width * r],
                  [-length * r, +length * r],
                  [0, 5]]
        World.__init__(self, bounds)
        self.bounds = bounds

        self.box = box(0, random_checkerboard(0.5), width, length)
       
        self.circles = [] 
        for i in range(self.num_circles):
            c = Circle(id_object=1 + i, tags=[],
                     texture=random_checkerboard(0.1),
                     center=[0, 0],
                     radius=1,
                     solid=True)
            self.circles.append(c)
            
        self.refresh()
        
    def refresh(self):
        for i in range(self.num_circles):
            x = np.random.uniform(self.bounds[0][0], self.bounds[0][1])
            y = np.random.uniform(self.bounds[1][0], self.bounds[1][1])
            radius = np.random.uniform(self.circles_size[0], self.circles_size[1])
            self.circles[i].center = [x, y]
            self.circles[i].radius = radius
        
    def get_primitives(self):
        return [self.box] + self.circles
    
    def new_episode(self):
        self.refresh()
        return World.new_episode(self)

    def simulate(self, dt, vehicle_pose):
        return []
