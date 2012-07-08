from . import World, np, random_checkerboard, box, contract
from . import Circle

__all__ = ['DynamicTest']


class DynamicTest(World):
    ''' A simple example of a dynamic world. '''

    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10):
        self.width = width
        self.length = length
        r = 1
        bounds = [[-width * r, +width * r],
                  [-length * r, +length * r],
                  [0, 5]]
        World.__init__(self, bounds)

        self.box = box(0, random_checkerboard(0.5), width, length)
        self.c1 = Circle(id_object=1, tags=[],
                             texture=random_checkerboard(0.1),
                             center=[width, length],
                             radius=0.5,
                             solid=True)
        self.c2 = Circle(id_object=2, tags=[],
                             texture=random_checkerboard(0.1),
                             center=[-width, -length],
                             radius=1.5,
                             solid=True)
        self.time = 0

    def get_primitives(self):
        return [self.box, self.c1, self.c2]

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        self.time += 1
        t = self.time
        omega = 0.1
        r = self.width / 2
        self.c1.center = [np.cos(t * omega) * r, np.sin(t * omega) * r]
        r2 = self.width / 3
        self.c2.center = [np.cos(t * omega) * r2, np.sin(t * omega) * r2]
        return [self.c1, self.c2]
