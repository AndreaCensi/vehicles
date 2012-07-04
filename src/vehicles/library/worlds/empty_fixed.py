from . import contract, np, World
from geometry import SE2_from_xytheta, SE3_from_SE2
from vehicles.utils import unique_timestamp_string

__all__ = ['EmptyFixed']


class EmptyFixed(World):
    """ Empty world; vehicle starts always at the same pose. """

    @contract(start_pose='seq[3](number)')
    def __init__(self, bounds, start_pose=[0, 0, 0]):
        World.__init__(self, bounds)
        self.start_pose = start_pose

    def get_primitives(self):
        return []

    def simulate(self, dt, vehicle_pose): #@UnusedVariable
        return []

    def new_episode(self):
        ''' Returns an episode tuple. By default it just
            samples a random pose for the robot.
        '''
        p = self.start_pose
        x = p[0]
        y = p[1]
        th = np.deg2rad(p[2])
        vehicle_state = SE3_from_SE2(SE2_from_xytheta([x, y, th]))
        id_episode = unique_timestamp_string()
        return World.Episode(id_episode, vehicle_state)
