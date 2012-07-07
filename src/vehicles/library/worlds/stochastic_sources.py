from . import Counter, contract
from . import World, Source
from vehicles.library.worlds.stochastic_box2 import KernelInverse

__all__ = ['StochasticSources']


class StochasticSources(World):
    ''' This also includes point sources. '''

    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10, num_sources=20, scale=2):
        r = 1
        bounds = [[-width * r, +width * r],
                  [-length * r, +length * r],
                  [0, 5]]
        World.__init__(self, bounds)

        id_object = Counter(1000)

        self.sources = []

        for _ in range(num_sources):
            c = Source(id_object=id_object(), tags=[],
                       center=[0, 0],
                       kernel_spec=KernelInverse.spec(scale, 1))
            self.sources.append(c)

        self.refresh()

    def refresh(self):
        for s in self.sources:
            s.set_center(self.random_2d_point())

    def get_primitives(self):
        return self.sources

    def new_episode(self):
        self.refresh()
        return World.new_episode(self)

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []  # no changes

