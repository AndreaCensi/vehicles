from . import Primitive, World, contract

__all__ = ['LightBox']


class LightBox(World):
    """ Parametric world """

    @contract(start_poses='None|list(seq[3](number))',
              primitives='list(dict)')
    def __init__(self, bounds, start_poses=None, primitives=[]):
        World.__init__(self, bounds, start_poses=start_poses)

        self.primitives = []
        self.cur_pose = None
        for x in primitives:
            x['surface'] = len(self.primitives)
            p = Primitive.from_yaml(x)
            self.primitives.append(p)

    def get_primitives(self):
        return self.primitives

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
