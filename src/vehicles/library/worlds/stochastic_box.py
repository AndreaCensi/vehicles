from . import random_checkerboard, box, np, contract, Counter
from . import Circle, World

__all__ = ['StochasticBox']


class StochasticBox(World):

    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10,
                 num_circles=15, circles_size=[1, 3]):
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

        id_object = Counter()

        self.box = box(id_object(), random_checkerboard(0.5), width, length)

        self.circles = []
        for _ in range(self.num_circles):
            c = Circle(id_object=id_object(),
                       tags=[],
                       texture=random_checkerboard(0.5),
                       center=[0, 0],
                       radius=1,
                       solid=True)
            self.circles.append(c)

        self.new_episode_called = False

    def refresh(self):
        for c in self.circles:
            c.set_center(self.random_2d_point())
            radius = np.random.uniform(self.circles_size[0],
                                       self.circles_size[1])
            c.radius = radius

    def get_primitives(self):
        if not self.new_episode_called:
            raise Exception('get_primitives() called before new_episode()')
        return [self.box] + self.circles

    def new_episode(self):
        self.new_episode_called = True
        self.refresh()
        return World.new_episode(self)

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
