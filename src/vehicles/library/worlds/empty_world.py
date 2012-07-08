from . import World

__all__ = ['Empty']


class Empty(World):

    def get_primitives(self):
        return []

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []
