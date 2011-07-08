from . import compute_collision
from collections import namedtuple
from contracts import contract
from geometry import SE2_identity, translation_from_SE2, SE3_from_SE2
import numpy as np
from vehicles.simulation.collision import collides_with

class Vehicle:
    
    def __init__(self, radius=0.5): # XXX
        self.radius = radius
        self.num_sensels = 0
        self.sensors = []
        self.id_sensors = None
        self.id_dynamics = None # XXX
        self.dynamics = None
        
        self.world = None
        
    def add_dynamics(self, id_dynamics, dynamics):
        assert self.dynamics is None, 'not sure if this will be implemented'
        self.dynamics = dynamics
        self.commands_spec = dynamics.commands_spec
        self.id_dynamics = id_dynamics
        # XXX: this is fishy
        self.state = self.dynamics.state_space().sample_uniform()
    
    AttachedSensor = namedtuple('AttachedSensor', 'sensor pose joint')
    def add_sensor(self, id_sensor, sensor, pose, joint):
        pose = self.dynamics.pose_space().from_yaml(pose) # XXX
        attached = Vehicle.AttachedSensor(sensor, pose, joint)
        self.sensors.append(attached)
        self.num_sensels += attached.sensor.num_sensels
        if not self.id_sensors:
            self.id_sensors = id_sensor
        else:
            self.id_sensors += '+%s' % id_sensor

    def set_world(self, world, updated=None):
        self.world = world
        for attached in self.sensors:
            attached.sensor.set_world(world, updated=updated)
            
    @contract(returns='SE3')
    def get_pose(self):
        ''' 
            Returns the pose of the robot in SE(3). 
            This is regardless of the state space.
            The idea is that all robot spaces are subgroups of SE(3)
            so this is the most general representation.
        '''
        # FIXME: make conversions 
        return SE3_from_SE2(self.state)
        
    def set_state(self, state):
        if self.world is None:
            raise ValueError('Please call set_world() before set_state().')
        # TODO: check compatibility
        self.dynamics.state_space().belongs(state)
        self.state = state
        
        collision = self.colliding_state(state)
        if collision.collided:
            raise ValueError('Cannot put the robot in a collding state')
        
    def simulate(self, commands, dt):
        # TODO: collisions
        primitives = self.world.get_primitives()
        def dynamics_function(t):
            s = self.dynamics.integrate(self.state, commands, t)
            # compute center of robot
            where = SE2_identity()
            pose = self.dynamics.compute_relative_pose(s, where)
            center = translation_from_SE2(pose)
            
            #print('t=%f, center=%s' % (t, center))
            return center
        
        collision = compute_collision(dynamics_function=dynamics_function,
                                      max_dt=dt,
                                      primitives=primitives, radius=self.radius)
        if collision.collided:
            #print('Collision at time %s' % collision.time)
            self.state = self.dynamics.integrate(self.state, commands, collision.time)
        else:
            self.state = self.dynamics.integrate(self.state, commands, dt)
        
        
    def compute_observations(self):
        # TODO: add dynamics observations
        sensel_values = []
        for attached in self.sensors:
            world_pose = self.dynamics.compute_relative_pose(
                            self.state, attached.pose, attached.joint)
            observations = attached.sensor.compute_observations(world_pose)
            sensels = observations['sensels']
            sensel_values.extend(sensels.tolist())
        return np.array(sensel_values)
        
    def colliding_state(self, state):
        ''' 
            Checks that the given state does not give collisions. 
            Returns None or a CollisionInfo structure. 
        '''
        pose = self.dynamics.compute_relative_pose(state, SE2_identity())
        center = translation_from_SE2(pose)
            
        collision = collides_with(self.world.get_primitives(),
                                  center, self.radius)
        return collision
