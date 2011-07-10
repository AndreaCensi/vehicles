from . import collides_with, np, compute_collision, contract
from collections import namedtuple
from geometry import  translation_from_SE2
from geometry import SE2_project_from_SE3


class Vehicle:
    
    def __init__(self, radius=0.5): # XXX
        self.radius = radius
        self.num_sensels = 0
        self.sensors = []
        self.id_sensors = None
        self.id_dynamics = None # XXX
        self.dynamics = None
        
        self.world = None
        
    def __repr__(self):
        return 'V(%s;%s)' % (self.id_dynamics, self.id_sensors)
        
    def add_dynamics(self, id_dynamics, dynamics):
        assert self.dynamics is None, 'not sure if this will be implemented'
        self.dynamics = dynamics
        self.commands_spec = dynamics.commands_spec
        self.id_dynamics = id_dynamics
        # XXX: this is fishy
        self.state = self.dynamics.state_space().sample_uniform()
    
    AttachedSensor = namedtuple('AttachedSensor', 'sensor pose joint')
    @contract(id_sensor='str', pose='SE3', joint='int,>=0')
    def add_sensor(self, id_sensor, sensor, pose, joint):
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
        j_pose, j_vel = self.dynamics.joint_state(self.state, 0) #@UnusedVariable
        return j_pose
        
    @contract(pose='SE3')
    def set_pose(self, pose):
        if self.world is None:
            raise ValueError('Please call set_world() before set_state().')
        # TODO: check compatibility
        state = self.dynamics.pose2state(pose)
        
        collision = self.colliding_pose(pose) # XXX
        if collision.collided:
            raise ValueError('Cannot put the robot in a collding state')
        
        self.state = state
        
    def simulate(self, commands, dt):
        # TODO: collisions
        primitives = self.world.get_primitives()
        def dynamics_function(t):
            state = self.dynamics.integrate(self.state, commands, t)
            # compute center of robot
            j_pose, j_vel = self.dynamics.joint_state(state, 0) #@UnusedVariable
            center = translation_from_SE2(SE2_project_from_SE3(j_pose))
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
        
        self.collision = collision
        
    def compute_observations(self):
        # TODO: add dynamics observations
        sensel_values = []
        for attached in self.sensors:
            j_pose, j_vel = self.dynamics.joint_state(self.state, attached.joint) #@UnusedVariable
            world_pose = np.dot(j_pose, attached.pose)
#            world_vel = None # XXX
            observations = attached.sensor.compute_observations(world_pose)
            sensels = observations['sensels']
            sensel_values.extend(sensels.tolist())
        return np.array(sensel_values)
        
    @contract(pose='SE3')
    def colliding_pose(self, pose):
        ''' 
            Checks that the given pose does not give collisions. 
            Returns None or a CollisionInfo structure. 
        '''
        state = self.dynamics.pose2state(pose)
        j_pose = self.dynamics.joint_state(state, 0)[0]
        center = translation_from_SE2(SE2_project_from_SE3(j_pose))
            
        collision = collides_with(self.world.get_primitives(),
                                  center, self.radius)
        return collision
