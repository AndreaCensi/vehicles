from . import collides_with, np, compute_collision, contract
from geometry import translation_from_SE2, SE2_project_from_SE3
from geometry.yaml import to_yaml


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
            return {
                'sensor': self.sensor.to_yaml(),
                'pose': to_yaml('SE3', self.pose),
                'joint': self.joint,
                'current_observations': dict_to_yaml(self.current_observations),
                'current_pose': to_yaml('SE3', self.current_pose),
            }
            
    def __init__(self, radius=0.5): # XXX
        self.radius = radius
        self.num_sensels = 0
        self.sensors = [] # array of Attached
        self.id_sensors = None
        self.id_dynamics = None # XXX
        self.dynamics = None
        
        self.primitives = set()
        
        self.state = None # First initialied in add_dynamics()
        
    def to_yaml(self):
        data = {
            'radius': self.radius,
            'num_sensels': self.num_sensels,
            'id_sensors': self.id_sensors,
            'id_dynamics': self.id_dynamics,
            'pose': to_yaml('SE3', self.dynamics.joint_state(self.state, 0)[0]),
            'conf': to_yaml('TSE3', self.dynamics.joint_state(self.state, 0)),
            'state': self.dynamics.state_to_yaml(self.state),
            'sensors': [s.to_yaml() for s in self.sensors],
        }
        return data
        
    def __repr__(self):
        return 'V(%s;%s)' % (self.id_dynamics, self.id_sensors)
        
    def add_dynamics(self, id_dynamics, dynamics):
        assert self.dynamics is None, 'not sure if this will be implemented'
        self.dynamics = dynamics
        self.commands_spec = dynamics.commands_spec
        self.id_dynamics = id_dynamics
        # XXX: this is fishy
        self.state = self.dynamics.state_space().sample_uniform()
    
    
    @contract(id_sensor='str', pose='SE3', joint='int,>=0')
    def add_sensor(self, id_sensor, sensor, pose, joint):
        attached = Vehicle.Attached(sensor, pose, joint)
        self.sensors.append(attached)
        self.num_sensels += attached.sensor.num_sensels
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
        pose = self.dynamics.joint_state(self.state, 0)
        j_pose, j_vel = pose #@UnusedVariable
        return j_pose
        
    @contract(pose='SE3')
    def set_pose(self, pose):
        if self.primitives is None:
            msg = 'Please call set_world_primitives() before set_state().'
            raise ValueError(msg)
        # TODO: check compatibility
        state = self.dynamics.pose2state(pose)
        
        collision = self.colliding_pose(pose) # XXX
        if collision.collided:
            raise ValueError('Cannot put the robot in a collding state')
        
        self.state = state
        
    def simulate(self, commands, dt):
        # TODO: collisions
        def dynamics_function(t):
            state = self.dynamics.integrate(self.state, commands, t)
            # compute center of robot
            j_pose, j_vel = self.dynamics.joint_state(state, 0) #@UnusedVariable
            center = translation_from_SE2(SE2_project_from_SE3(j_pose))
            #print('t=%f, center=%s' % (t, center))
            return center
        
        collision = compute_collision(dynamics_function=dynamics_function,
                                      max_dt=dt,
                                      primitives=self.primitives,
                                      radius=self.radius)
        if collision.collided:
            #print('Collision at time %s' % collision.time)
            self.state = self.dynamics.integrate(self.state, commands,
                                                 collision.time)
        else:
            self.state = self.dynamics.integrate(self.state, commands, dt)
        
        self.collision = collision
        
    def compute_observations(self):
        # TODO: add dynamics observations
        sensel_values = []
        for attached in self.sensors:
            pose = self.dynamics.joint_state(self.state, attached.joint)
            j_pose, j_vel = pose #@UnusedVariable
            attached.current_pose = np.dot(j_pose, attached.pose)
            attached.current_observations = \
                attached.sensor.compute_observations(attached.current_pose)
            sensels = attached.current_observations['sensels']
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
            
        collision = collides_with(self.primitives,
                                  center, self.radius)
        return collision



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
