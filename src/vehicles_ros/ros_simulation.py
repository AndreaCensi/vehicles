from . import (publish_world, publish_vehicle, numpy_to_imgmsg,)
from bootstrapping_olympics import RobotInterface
from contracts import contract
from pprint import pformat
from vehicles import (instance_vehicle_spec, instance_world_spec,
    check_valid_simulation_config, instance_vehicle, instance_world,
    VehicleSimulation)
import contracts
import numpy as np
import rospy #@UnresolvedImport
import yaml

class VizLevel:
    # Visualization levels
    Nothing = 0
    State = 0 # TODO: reconsider
    Geometry = 1
    Sensels = 2
    SensorData = 3
    Everything = 3
    
class ROSVehicleSimulation(RobotInterface, VehicleSimulation):
    
    def __init__(self, **params):
        contracts.disable_all()
        rospy.loginfo('Received configuration:\n%s' % pformat(params))
        check_valid_simulation_config(params)
        
        self.dt = params.get('dt', 0.1)
        
        if 'vehicle' in params:
            id_vehicle = params['vehicle']['id']
            vehicle = instance_vehicle_spec(params['vehicle'])
        else:
            id_vehicle = params['id_vehicle']
            vehicle = instance_vehicle(id_vehicle)
            
        if 'world' in params:
            id_world = params['world']['id']
            world = instance_world_spec(params['world'])
        else:
            id_world = params['id_vehicle']
            world = instance_world(id_world)
        
        VehicleSimulation.__init__(self, vehicle, world)
        
        commands_spec = self.vehicle.commands_spec
        observations_shape = self.vehicle.num_sensels
        
        RobotInterface.__init__(self,
                 observations_shape, commands_spec,
                 id_robot=id_vehicle,
                 id_sensors=self.vehicle.id_sensors,
                 id_actuators=self.vehicle.id_dynamics)
        
        self.viz_level = params.get('viz_level', VizLevel.Everything)
        
        # TODO: make parameter

        if self.viz_level > VizLevel.Nothing:
            from . import Marker, Image
            self.publisher = rospy.Publisher('~markers', Marker)
            self.pub_sensels_image = rospy.Publisher('~sensels_image', Image)
            self.pub_commands_image = rospy.Publisher('~commands_image', Image)
            self.first_time = True
            
        if self.viz_level >= VizLevel.State:
            from . import  String
            self.pub_state = rospy.Publisher('~state', String)
            
    def info(self, s):
        rospy.loginfo(s)
           
    def __repr__(self):
        return 'VehicleSimulation(%s,%s)' % (self.id_vehicle, self.id_world)

    def set_commands(self, commands):
        VehicleSimulation.simulate(self, commands, self.dt)
        if self.viz_level >= VizLevel.Sensels:
            self.publish_ros_commands(commands)
        if self.viz_level >= VizLevel.Geometry:
            self.publish_ros_markers()
            
        # TODO: add level?
        if self.viz_level >= VizLevel.State:
            from . import String
            y = self.to_yaml()
            s = yaml.dump(y) 
            self.pub_state.publish(String(s))
            
        if self.vehicle_collided:
            rospy.loginfo('Restarting new episode due to collision.')
            self.new_episode()
            
    def get_observations(self):
        observations = VehicleSimulation.compute_observations(self)
        if self.viz_level >= VizLevel.Sensels:
            self.publish_ros_sensels(observations)
    
        return self.timestamp, observations
        
    def new_episode(self):
        return VehicleSimulation.new_episode(self) 
    
    def publish_ros_markers(self):
        plot_params = dict(
            points_width=0.03,
            z_sensor=0.75,
            world_frame='/world',
            stamp=rospy.get_rostime(),
            visualize_sensors=self.viz_level >= VizLevel.SensorData,
            z0=0.0,
            z1=1.0,
            z_sensor_width=1.0,
            robot_height=1.0,
        ) 
        
        publish_world(self.publisher, plot_params, self.world)
        publish_vehicle(self.publisher, plot_params, self.vehicle)
    
    def publish_ros_commands(self, commands):
        from reprep import posneg
        #commands = commands.reshape((1, commands.size))
        z = 4
        commands = np.kron(commands, np.ones((z, z)))
        commands_image = posneg(commands)
        ros_image = numpy_to_imgmsg(commands_image, stamp=None)
        self.pub_commands_image.publish(ros_image)
        
    def publish_ros_sensels(self, obs):
        from reprep import scale
        obs2d = reshape_smart(obs)
        obs2d_image = scale(obs2d)
        ros_image = numpy_to_imgmsg(obs2d_image, stamp=None)
        self.pub_sensels_image.publish(ros_image)
        
# TODO: move somewhere
@contract(x='array')
def reshape_smart(x, width=None): # TODO: move
    ''' Reshapes x into (?, width) if x is 1D.
    
        If x is 2D, it is left alone.
    '''
    if x.ndim == 2:
        return x

    n = x.size
    
    if width is None:
        width = np.ceil(np.sqrt(n))
    
    height = np.ceil(n * 1.0 / width)
    
    #print("sha: %s  n: %d  with: %d  height: %d" % (x.shape,n,width,height))
    
    y = np.zeros(shape=(height, width), dtype=x.dtype)
    y.flat[0:n] = x
    y.flat[n:] = np.nan
    
    return y  
            
