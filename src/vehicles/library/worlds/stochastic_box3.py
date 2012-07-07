from . import box, np, contract, Counter
from . import Circle, World
from .utils import random_checkerboard_smooth

__all__ = ['StochasticBox3']


class StochasticBox3(World):

    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10,
                        num_circles=15, circles_size=[1, 3],
                       scale=1, sigma=1):
        self.width = width
        self.length = length

        self.circles_size = circles_size
        r = 1
        bounds = [[-width * r, +width * r],
                  [-length * r, +length * r],
                  [0, 5]]
        World.__init__(self, bounds)

        texture = lambda: random_checkerboard_smooth(scale, sigma)

        id_object = Counter()

        self.box = box(id_object(), texture(), width, length)

        self.circles = []
        for _ in range(num_circles):
            c = Circle(id_object=id_object(), tags=[],
                     texture=texture(),
                     center=[0, 0],
                     radius=1,
                     solid=True)
            self.circles.append(c)

        self.refresh()

    def refresh(self):
        for c in self.circles:
            p = self.random_2d_point()
            radius = np.random.uniform(self.circles_size[0],
                                       self.circles_size[1])
            c.set_center(p)
            c.radius = radius

    def get_primitives(self):
        return [self.box] + self.circles

    def new_episode(self):
        self.refresh()
        return World.new_episode(self)

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
