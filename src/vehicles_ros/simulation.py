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

class VizLevel:
    # Visualization levels
    Nothing = 0
    Geometry = 1
    Sensels = 2
    SensorData = 3
    Everything = 3
    
class ROSVehicleSimulation(RobotInterface, VehicleSimulation):

    
    def __init__(self, **params):
        contracts.disable_all()
        rospy.loginfo('Received configuration:\n%s' % pformat(params))
        check_valid_simulation_config(params)
        
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
            
    def info(self, s):
        rospy.loginfo(s)
           
    def __repr__(self):
        return 'VehicleSimulation(%s,%s)' % (self.id_vehicle, self.id_world)

    def set_commands(self, commands):
        dt = 0.1 # XXX
        VehicleSimulation.simulate(self, commands, dt)
        if self.viz_level >= VizLevel.Sensels:
            self.publish_ros_commands(commands)
        if self.viz_level >= VizLevel.Geometry:
            self.publish_ros_markers()
            
        if self.vehicle_collided:
            rospy.loginfo('Restarting new episode due to collision.')
            self.new_episode()
            
    def get_observations(self):
        observations = VehicleSimulation.compute_observations(self)
        if self.viz_level >= VizLevel.Sensels:
            self.publish_ros_sensels(observations)
    
        return observations
        
    def new_episode(self):
        return VehicleSimulation.new_episode(self) 
    
    def publish_ros_markers(self):
#        vehicle_pose = self.vehicle.get_pose()
#        if False: 
#            br = tf.TransformBroadcaster()
#            br.sendTransform((0, 0, 0),
#                             tf.transformations.quaternion_from_euler(0, 0, 0),
#                             rospy.Time.now(),
#                             "world",
#                             "/map")
#        
#            rotation, translation = rotation_translation_from_pose(vehicle_pose)
#            q = ROS_quaternion_order(quaternion_from_rotation(rotation))
#            br.sendTransform((translation[0], translation[1], translation[2]),
#                             (q[0], q[1], q[2], q[3]),
#                             rospy.Time.now(),
#                             "vehicle_pose",
#                             "world")
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
        
        

@contract(x='array')
def reshape_smart(x, width=None):
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

        
            
