from .ros_utils import ROS_quaternion_order, numpy_to_imgmsg
from bootstrapping_olympics import RobotSimulationInterface
from contracts import contract
from geometry import quaternion_from_rotation, rotation_translation_from_pose
from vehicles import PolyLine, Circle, instance_vehicle, instance_world
import numpy as np
from vehicles import VehicleSimulation

try:
    import rospy #@UnresolvedImport
    visualization = True
except:
    visualization = False
    
class ROSVehicleSimulation(RobotSimulationInterface, VehicleSimulation):
    
    def __init__(self, id_vehicle, id_world):
        self.id_vehicle = id_vehicle
        self.id_world = id_world
        
        vehicle = instance_vehicle(id_vehicle)
        world = instance_world(id_world)
        
        VehicleSimulation.__init__(vehicle, world)
        
        commands_spec = self.vehicle.commands_spec
        observations_shape = self.vehicle.num_sensels
        
        RobotSimulationInterface.__init__(self,
                 observations_shape, commands_spec,
                 id_robot=id_vehicle,
                 id_sensors=self.vehicle.id_sensors,
                 id_actuators=self.vehicle.id_dynamics)
        
        if visualization:
            from visualization_msgs.msg import Marker #@UnresolvedImport
            from sensor_msgs.msg import Image #@UnresolvedImport
            self.publisher = rospy.Publisher('~markers', Marker)
            self.pub_sensels_image = rospy.Publisher('~sensels_image', Image)
            self.pub_commands_image = rospy.Publisher('~commands_image', Image)
            self.first_time = True
            
    def __repr__(self):
        return 'VehicleSimulation(%s,%s)' % (self.id_vehicle, self.id_world)

    @contract(commands='array[K]')
    def simulate(self, commands, dt):
        VehicleSimulation.simulate(self, commands, dt)
        if visualization:
            self.publish_ros_commands(commands)
            self.publish_ros_markers()
            
    def compute_observations(self):
        observations = VehicleSimulation.compute_observations()
        if visualization:
            self.publish_ros_sensels(observations)
    
        return observations
        
    def new_episode(self):
        return VehicleSimulation.new_episode()
#        episode = self.world.new_episode()
#        self.id_episode = episode.id_episode
#        self.vehicle.set_state(episode.vehicle_state)
#        self.vehicle.set_world(self.world)
#        return episode
    
    def publish_ros_markers(self): 
        from visualization_msgs.msg import Marker #@UnresolvedImport
        from geometry_msgs.msg import Point #@UnresolvedImport
        import tf #@UnresolvedImport
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "world",
                         "/map")
    
        vehicle_pose = self.vehicle.get_pose()
        rotation, translation = rotation_translation_from_pose(vehicle_pose)
        q = ROS_quaternion_order(quaternion_from_rotation(rotation))
        br.sendTransform((translation[0], translation[1], translation[2]),
                         (q[0], q[1], q[2], q[3]),
                         rospy.Time.now(),
                         "vehicle_pose",
                         "world")
    
        marker = Marker()
        marker.header.frame_id = 'vehicle_pose'
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.publisher.publish(marker)
        
        if self.first_time:
            self.first_time = False
        
        for x in self.world.get_primitives():
            surface = x.id_object
            if isinstance(x, PolyLine):
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp = rospy.get_rostime()
                marker.ns = "world"
                marker.id = surface
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                points_list = [
                               Point(x[0], x[1], 0) for
                               x in x.points]
                marker.points = points_list
                marker.scale.x = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                self.publisher.publish(marker)
        
            elif isinstance(x, Circle):
                pass
            else: 
                raise Exception('Unexpected object %s' % x)

    
    @contract(commands='array[K]')
    def publish_ros_commands(self, commands):
        from reprep.graphics.posneg import posneg
        #commands = commands.reshape((1, commands.size))
        z = 4
        commands = np.kron(commands, np.ones((z, z)))
        commands_image = posneg(commands)
        ros_image = numpy_to_imgmsg(commands_image, stamp=None)
        self.pub_commands_image.publish(ros_image)
        
    def publish_ros_sensels(self, obs):
        from reprep.graphics.scale import scale
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

        
            
