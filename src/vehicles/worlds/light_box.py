from . import contract, np
from ..interfaces import Primitive, World, unique_timestamp_string
from geometry import SE2_from_xytheta, SE3_from_SE2


__all__ = ['LightBox']


class LightBox(World):
    """ Parametric world """

    @contract(start_poses='None|list(seq[3](number))',
              primitives='list(dict)')
    def __init__(self, bounds, start_poses=None, primitives=[]):
        World.__init__(self, bounds)
        self.start_poses = start_poses
        self.primitives = []
        self.cur_pose = None
        for x in primitives:
            x['surface'] = len(self.primitives)
            p = Primitive.from_yaml(x)
            self.primitives.append(p)

    def get_primitives(self):
        return self.primitives

    def simulate(self, dt, vehicle_pose):
        return []

    def new_episode(self):
        ''' Returns an episode tuple. By default it just
            samples a random pose for the robot.
        '''
        if self.start_poses is None:
            x = np.random.uniform(self.bounds[0][0], self.bounds[0][1])
            y = np.random.uniform(self.bounds[1][0], self.bounds[1][1])
            th = np.random.uniform(0, np.pi * 2)
        else:
            nposes = len(self.start_poses)
            if self.cur_pose is None:
                self.cur_pose = np.random.randint(nposes)
            else:
                self.cur_pose = (self.cur_pose + 1) % nposes
            p = self.start_poses[self.cur_pose]
            x = p[0]
            y = p[1]
            th = np.deg2rad(p[2])
        vehicle_state = SE3_from_SE2(SE2_from_xytheta([x, y, th]))
        id_episode = unique_timestamp_string()
        return World.Episode(id_episode, vehicle_state)
