from . import collides_with, np, compute_collision, contract
from geometry import translation_from_SE2, SE2_project_from_SE3
from geometry.yaml import to_yaml
from geometry.manifolds import SE2, SE3
from geometry.poses import SE2_from_SE3


class Vehicle:
    class Attached:
        @contract(pose='SE3', joint='int')
        def __init__(self, sensor, pose, joint):
            self.sensor = sensor
            self.pose = pose
            self.joint = joint
            self.current_observations = None
            self.current_pose = None

        def to_yaml(self):
            if self.current_observations is None:
                raise Exception('Observations not computed')
#            print(' serializing: rel pose: %s'
#                  % SE2.friendly(SE2_from_SE3(self.pose)))
#            print(' serializing: current pose: %s'
#                  % SE2.friendly(SE2_from_SE3(self.current_pose)))
            return {
                'sensor': self.sensor.to_yaml(),
                'pose': to_yaml('SE3', self.pose),
                'joint': self.joint,
                'current_observations':
                    dict_to_yaml(self.current_observations),
                'current_pose': to_yaml('SE3', self.current_pose),
            }

    def __init__(self, radius=0.5):
        self.radius = radius
        self.sensors = []  # array of Attached
        self.id_sensors = None
        self.id_dynamics = None  # XXX
        self.dynamics = None

        self.primitives = set()

        # Needs to be initialized before calling certain functions
        self._state = None

    def to_yaml(self):
        # pose, velocity
        configuration = self.dynamics.joint_state(self._get_state(), 0)
        pose = configuration[0]
        #print ('Serializing robot_pose: %s' % SE2.friendly(SE2_from_SE3(pose)))
        data = {
            'radius': self.radius,
            'id_sensors': self.id_sensors,
            'id_dynamics': self.id_dynamics,
            'pose': to_yaml('SE3', pose),
            'conf': to_yaml('TSE3', configuration),
            'state': self.dynamics.state_to_yaml(self._get_state()),
            'sensors': [s.to_yaml() for s in self.sensors],
        }
        return data

    def __repr__(self):
        return 'V(%s;%s)' % (self.id_dynamics, self.id_sensors)

    def add_dynamics(self, id_dynamics, dynamics):
        assert self.dynamics is None, 'not sure if this will be implemented'
        self.dynamics = dynamics
        self.id_dynamics = id_dynamics

    @contract(id_sensor='str', pose='SE3', joint='int,>=0')
    def add_sensor(self, id_sensor, sensor, pose, joint):
        attached = Vehicle.Attached(sensor, pose, joint)
        self.sensors.append(attached)
        if not self.id_sensors:
            self.id_sensors = id_sensor
        else:
            self.id_sensors += '+%s' % id_sensor

    def set_world_primitives(self, primitives):
        # These are used for collision checking
        self.primitives.update(primitives)
        for attached in self.sensors:
            attached.sensor.set_world_primitives(primitives)

    @contract(returns='SE3')
    def get_pose(self):
        ''' 
            Returns the pose of the robot in SE(3). 
            This is regardless of the state space.
            The idea is that all robot spaces are subgroups of SE(3)
            so this is the most general representation.
        '''
        pose = self.dynamics.joint_state(self._get_state(), 0)
        j_pose, j_vel = pose  # @UnusedVariable
        return j_pose

    @contract(pose='SE3')
    def set_pose(self, pose):
        if self.primitives is None:
            msg = 'Please call set_world_primitives() before set_pose().'
            raise ValueError(msg)
        # TODO: check compatibility
        state = self.dynamics.pose2state(pose)

        collision = self.colliding_pose(pose)  # XXX
        if collision.collided:
            raise ValueError('Cannot put the robot in a colliding state.')

        self._state = state

        # TODO: invalidate observations?

    def simulate(self, commands, dt):
        # TODO: collisions
        def dynamics_function(t):
            state = self.dynamics.integrate(self._get_state(), commands, t)
            # compute center of robot
            j_pose, _ = self.dynamics.joint_state(state, 0)
            center = translation_from_SE2(SE2_project_from_SE3(j_pose))
            #print('t=%f, center=%s' % (t, center))
            return center

        collision = compute_collision(dynamics_function=dynamics_function,
                                      max_dt=dt,
                                      primitives=self.primitives,
                                      radius=self.radius)
        if collision.collided:
            #print('Collision at time %s' % collision.time)
            next_state = self.dynamics.integrate(self._get_state(), commands,
                                                 collision.time)
        else:
            next_state = self.dynamics.integrate(self._get_state(), commands,
                                                 dt)

        self._state = next_state
        self.collision = collision

    def compute_observations(self):
        # TODO: add dynamics observations
        sensel_values = []
        for attached in self.sensors:
            pose = self.dynamics.joint_state(self._get_state(), attached.joint)
            j_pose, _ = pose
            #print('joint pose: %s' % SE2.friendly(SE2_from_SE3(j_pose)))
            #print('  relative: %s' % SE2.friendly(SE2_from_SE3(attached.pose)))
            attached.current_pose = SE3.multiply(j_pose, attached.pose)
            #print('   current: %s' % SE2.friendly(SE2_from_SE3(attached.current_pose)))
            attached.current_observations = \
                attached.sensor.compute_observations(attached.current_pose)
            sensels = attached.current_observations['sensels']
            sensel_values.extend(sensels.tolist())
        return np.array(sensel_values, dtype='float32')

    @contract(pose='SE3')
    def colliding_pose(self, pose):
        ''' 
            Checks that the given pose does not give collisions. 
            Returns None or a CollisionInfo structure. 
        '''
        state = self.dynamics.pose2state(pose)
        j_pose = self.dynamics.joint_state(state, 0)[0]
        center = translation_from_SE2(SE2_project_from_SE3(j_pose))

        collision = collides_with(self.primitives,
                                  center, self.radius)
        return collision

    def _get_state(self):
        ''' Returns the dynamics state or raises an exception if it has
            not been set. This is an opaque object interpretable by 
            the Dynamics instance. '''
        if self._state is None:
            raise ValueError('The vehicle state has not been set yet.')
        return self._state


# TODO: move somewhere
def array_to_yaml(x):
    return x.tolist()


def dict_to_yaml(x):
    ''' Sanitizes the values in the dictionary. '''
    x = x.copy()
    for k in x:
        v = x[k]
        if isinstance(v, np.ndarray):
            x[k] = array_to_yaml(v)
    return x
