from ..utils import check_yaml_friendly
from .collision import collides_with, compute_collision
from contracts import contract
from geometry import (SE2_from_translation_angle, SE2_project_from_SE3, SE3, 
    SE3_from_SE2, translation_from_SE2)
from geometry.yaml import to_yaml
from vehicles import get_conftools_dynamics, get_conftools_sensors
import numpy as np
from geometry.manifolds import SE2
from geometry.poses import SE2_from_SE3

__all__ = [
    'Attached', 
    'Vehicle',
]


class Attached(object):
    
    @contract(pose='SE3', joint='int,>=0', extra='dict')
    def __init__(self, sensor, pose, joint, extra):
        '''
            Initializes this structure.
            
            :param extra: dict convertible to YAML.
             extra['skin']: sing for this sensor.
        '''
        self.sensor = sensor
        self.pose = pose
        self.joint = joint
        self.current_observations = None
        self.current_pose = None
        self.extra = extra

    def to_yaml(self):
        if self.current_observations is None:
            raise Exception('Observations not computed')
        return {
            'sensor': self.sensor.to_yaml(),
            'pose': SE3.to_yaml(self.pose),
            'joint': self.joint,
            'extra': self.extra,
            'current_observations':
                dict_to_yaml(self.current_observations),
            'current_pose': SE3.to_yaml(self.current_pose),
        }


class Vehicle(object):

    @contract(radius='>0', 
              dynamics='str|code_spec|isinstance(Dynamics)',
              sensors='list(dict)',
              extra='dict')
    def __init__(self, radius, dynamics, sensors, extra={}):
        """ 
            Initializes an empty vehicle.
            
            :param radius: radius (used for collision detection)
            :param extra: extra information (extra['skin'] id of the skin)
        """
        self.radius = radius
        self.sensors = []  # array of Attached
    
        _, self.dynamics = get_conftools_dynamics().instance_smarter(dynamics)
        self.extra = extra

        self.primitives = set()

        for sensor in sensors:
            if not 'sensor' in sensor:
                msg = 'Expected field "sensor": %s' % sensors
                raise ValueError(msg)
            config = get_conftools_sensors()
            _, sensor_instance = config.instance_smarter(sensor['sensor'])

            # TODO: document this
            x, y, theta_deg = sensor.get('pose', [0, 0, 0])
            theta = np.radians(theta_deg)
            pose = SE2_from_translation_angle([x, y], theta)
            pose = SE3_from_SE2(pose)
            joint = sensor.get('joint', 0)
            extra = sensor.get('extra', {})
            self.add_sensor(sensor=sensor_instance,
                            pose=pose, joint=joint,
                            extra=extra)

        # Needs to be initialized before calling certain functions
        self._state = None

    # @contract(returns='TSE3')
    def get_configuration(self):
        """ Returns pose/velocity of the platform """
        return self.dynamics.joint_state(self._get_state(), 0)
    
    def to_yaml(self):
        # pose, velocity
        configuration = self.get_configuration()
        pose = configuration[0]

        joints = []
        for i in range(self.dynamics.num_joints()):
            jc = self.dynamics.joint_state(self._get_state(), i)
            joints.append(to_yaml("TSE3", jc))

        data = {
            'radius': self.radius,
            'pose': SE3.to_yaml(pose),
            'conf': to_yaml('TSE3', configuration),
            'state': self.dynamics.state_to_yaml(self._get_state()),
            'joints': joints,
            'sensors': [s.to_yaml() for s in self.sensors],
            'extra': self.extra
        }
        check_yaml_friendly(data)
        return data

    def __repr__(self):
        return 'Vehicle()' 

    @contract( pose='SE3', joint='int,>=0')
    def add_sensor(self, sensor, pose, joint, extra):
        attached = Attached(sensor=sensor, pose=pose,
                            joint=joint, extra=extra)
        self.sensors.append(attached)

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
            msg = 'Cannot put the robot in a colliding state.'
            msg +='\n collision: %s ' % str(collision)
            msg +='\n pose: %s ' % SE2.friendly(SE2_from_SE3(pose))
            raise ValueError(msg)

        self._state = state

        # TODO: invalidate observations?

    def simulate(self, commands, dt):
        # TODO: collisions
        def dynamics_function(t):
            state = self.dynamics.integrate(self._get_state(), commands, t)
            # compute center of robot
            j_pose, _ = self.dynamics.joint_state(state, 0)
            center = translation_from_SE2(SE2_project_from_SE3(j_pose))
            # print('t=%f, center=%s' % (t, center))
            return center

        collision = compute_collision(dynamics_function=dynamics_function,
                                      max_dt=dt,
                                      primitives=self.primitives,
                                      radius=self.radius)
        if collision.collided:
            # print('Collision at time %s' % collision.time)
            next_state = self.dynamics.integrate(self._get_state(), commands,
                                                 collision.time)
        else:
            next_state = self.dynamics.integrate(self._get_state(), commands,
                                                 dt)

        self._state = next_state
        self.collision = collision

    @contract(returns='array')
    def compute_observations(self):
        # TODO: add dynamics observations
        sensel_values = []
        for attached in self.sensors:
            pose = self.dynamics.joint_state(self._get_state(), attached.joint)
            j_pose, _ = pose
            attached.current_pose = SE3.multiply(j_pose, attached.pose)
            attached.current_observations = \
                attached.sensor.compute_observations(attached.current_pose)
            sensels = attached.current_observations['sensels']
            sensel_values.extend(sensels.tolist())
        return np.array(sensel_values, dtype='float32')

    @contract(returns='dict')
    def compute_observations_sensor(self, index):
        '''
        Computes the observations for the sensor. Returns a dictionary
        with many fields; one of them is 'sensel'.
        
        :param index: Index of the sensor attached to the vehicle.
        '''
        attached = self.sensors[index]
        pose = self.dynamics.joint_state(self._get_state(), attached.joint)
        j_pose, _ = pose
        attached.current_pose = SE3.multiply(j_pose, attached.pose)
        attached.current_observations = \
            attached.sensor.compute_observations(attached.current_pose)
        return attached.current_observations


    @contract(pose='SE3')
    def colliding_pose(self, pose, safety_margin=1):
        ''' 
            Checks that the given pose does not give collisions. 
            Returns None or a CollisionInfo structure. 
            
            safety_margin: multiplies the radius
        '''
        state = self.dynamics.pose2state(pose)
        j_pose = self.dynamics.joint_state(state, 0)[0]
        center = translation_from_SE2(SE2_project_from_SE3(j_pose))

        collision = collides_with(self.primitives,
                                  center, self.radius * safety_margin)
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

    check_yaml_friendly(x)
    return x
