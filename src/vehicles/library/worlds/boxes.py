from . import World, PolyLine, random_checkerboard, np, contract


class Box(World):
    ''' A simple box. '''

    @contract(width='>0', length='>0')
    def __init__(self, width=10, length=10):
        r = 1
        bounds = [[-width * r, +width * r],
                  [-length * r, +length * r], [0, 5]]
        World.__init__(self, bounds)
        self.width = width
        self.length = length
        texture = random_checkerboard(0.5)
        points = [[-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
        points = [(np.array(p) * np.array([width, length])).tolist()
                  for p in points]
        self.box = PolyLine(id_object=0, tags=[], texture=texture,
                            points=points)

    def get_primitives(self):
        return [self.box]

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
