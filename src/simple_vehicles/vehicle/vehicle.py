from collections import namedtuple
import numpy as np
from contracts.main import contract
from geometry.poses import SE3_from_SE2

class Vehicle:
    
    def __init__(self):
        self.num_sensels = 0
        self.sensors = []
        self.id_sensors = None
        self.id_dynamics = None # XXX
        self.dynamics = None
        
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

    def set_world(self, world):
        self.world = world
        for attached in self.sensors:
            attached.sensor.set_world(world, updated=None)
            
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
        # TODO: check compatibility
        self.dynamics.state_space().belongs(state)
        self.state = state
        
    def simulate(self, commands, dt):
        # TODO: collisions
        self.state = self.dynamics.integrate(self.state, commands, dt)
        
        updated = self.world.simulate(dt)
        if len(updated) > 0: 
            for attached in self.sensors:
                attached.sensor.set_world(self.world, updated=updated)
        
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
        
