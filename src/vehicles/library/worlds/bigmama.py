from . import Circle, World, contract, Counter
from .utils import blackwhite_checkerboard

__all__ = [
    'BigMamarama',
]


class BigMamarama(World):

    @contract(radius='>0', scale='>0')
    def __init__(self, radius=10, scale=1):
        r = 1.1
        bounds = [[-radius * r, +radius * r],
                  [-radius * r, +radius * r],
                  [0, 5]]
        World.__init__(self, bounds, start_poses=[[0,0,0]])
        
        id_object = Counter()

        self.prim = []
        
        self.prim.append(
            Circle(id_object=id_object(),
                   tags=[],
                   texture=blackwhite_checkerboard(scale),
                   center=[0, 0],
                   radius=radius,
                   solid=False)
        )
#         
#         for i in range(10):
#             c = Circle(id_object=id_object(),
#                        tags=[],
#                        texture=random_checkerboard(0.5),
#                        center=[-0.4+i*0.1, 0.5],
#                        radius=1,
#                        solid=True)
#             self.prim.append(c)


    def get_primitives(self):
        return self.prim

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
