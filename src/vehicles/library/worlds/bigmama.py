from . import Circle, World, contract, Counter
from .utils import blackwhite_checkerboard

__all__ = ['BigMamarama']


class BigMamarama(World):

    @contract(radius='>0', scale='>0')
    def __init__(self, radius=10, scale=1):
        r = 1.1
        bounds = [[-radius * r, +radius * r],
                  [-radius * r, +radius * r],
                  [0, 5]]
        World.__init__(self, bounds)
        self.bounds = bounds

        id_object = Counter()

        self.circle = Circle(id_object=id_object(),
                       tags=[],
                       texture=blackwhite_checkerboard(scale),
                       center=[0, 0],
                       radius=radius,
                       solid=False)

    def get_primitives(self):
        return [self.circle]

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
