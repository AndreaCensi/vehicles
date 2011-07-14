from .ros_utils import ROS_quaternion_order, numpy_to_imgmsg
from bootstrapping_olympics import RobotSimulationInterface
from contracts import contract
from geometry import quaternion_from_rotation, rotation_translation_from_pose
from pprint import pformat
from vehicles import (PolyLine, Circle, instance_vehicle, instance_world,
    VehicleSimulation)
from vehicles.configuration.checks import check_valid_simulation_config
from vehicles.configuration.instance_all import (instance_vehicle_spec,
    instance_world_spec)
import contracts
import numpy as np
import rospy #@UnresolvedImport
import warnings
from vehicles_ros.ros_utils import ROS_Pose_from_SE3
from vehicles.sensors.raytracer.textured_raytracer import Raytracer
from geometry.poses import SE3_from_rotation_translation
from vehicles_ros.ros_plot import get_marker_for_world_extent

try:
    from ros import visualization_msgs
    from ros import sensor_msgs
    from ros import geometry_msgs
     
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker #@UnresolvedImport
    from sensor_msgs.msg import Image #@UnresolvedImport
    from geometry_msgs.msg import Point
    visualization = True
except ImportError as e:
    msg = """ROS Visualization packages (visualization_msgs, sensor_msgs, tf)
not installed; not visualizing anything. 
             
Error: %s             
        """ % e
    warnings.warn(msg)
    visualization = False
    
visualization = False

class ROSVehicleSimulation(RobotSimulationInterface, VehicleSimulation):
    
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
        
        RobotSimulationInterface.__init__(self,
                 observations_shape, commands_spec,
                 id_robot=id_vehicle,
                 id_sensors=self.vehicle.id_sensors,
                 id_actuators=self.vehicle.id_dynamics)
        
        if visualization:
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
            
        if self.vehicle.collision.collided:
            rospy.loginfo('Restarting new episode due to collision.')
            self.new_episode()
            
    def compute_observations(self):
        observations = VehicleSimulation.compute_observations(self)
        if visualization:
            self.publish_ros_sensels(observations)
    
        return observations
        
    def new_episode(self):
        return VehicleSimulation.new_episode(self) 
    
    def publish_ros_markers(self):
        vehicle_pose = self.vehicle.get_pose()
          
        if False: 
            br = tf.TransformBroadcaster()
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "world",
                             "/map")
        
            rotation, translation = rotation_translation_from_pose(vehicle_pose)
            q = ROS_quaternion_order(quaternion_from_rotation(rotation))
            br.sendTransform((translation[0], translation[1], translation[2]),
                             (q[0], q[1], q[2], q[3]),
                             rospy.Time.now(),
                             "vehicle_pose",
                             "world")
    
        z0 = 0.0
        z1 = 1.0
        z_sensor = 0.5
        z_sensor_width = 1.0
        robot_height = 1.0
        points_width = 0.03

        frame_id = '/world'
        stamp = rospy.get_rostime() # TODO: sim time
        
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        pose_marker = SE3_from_rotation_translation(
                                np.eye(3),
                                np.array([0, 0, robot_height / 2]))
        marker.pose = ROS_Pose_from_SE3(np.dot(vehicle_pose, pose_marker))
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = robot_height
        marker.color = ColorRGBA(1.0, 0.3, 0.3, 1.0)
        self.publisher.publish(marker)
        
        if self.first_time:
            self.first_time = False
        
        marker = get_marker_for_world_extent(self.world.bounds, frame_id, stamp)
        self.publisher.publish(marker)
        
            
        # TODO: only updated
        for x in self.world.get_primitives():
            surface = x.id_object
            if isinstance(x, PolyLine):
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = "world"
                marker.id = surface
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.ADD
                vertices = []
                for i in range(len(x.points) - 1):
                    p1 = x.points[i]
                    p2 = x.points[i + 1]
                    a = Point(p1[0], p1[1], z0) 
                    b = Point(p1[0], p1[1], z1)
                    c = Point(p2[0], p2[1], z0)
                    d = Point(p2[0], p2[1], z1)
                    vertices.extend([a, b, d])
                    vertices.extend([d, c, a])
                marker.points = vertices
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1
                marker.color = ColorRGBA(.5, .5, .2, 1)
                self.publisher.publish(marker)
        
            elif isinstance(x, Circle):
                pass
            else: 
                raise Exception('Unexpected object %s' % x) 
            
        for i, attached in enumerate(self.vehicle.sensors):
            sensor = attached.sensor
            observations = attached.current_observations
            world_pose = attached.current_pose
            if isinstance(sensor, Raytracer):
                directions = sensor.directions
                readings = observations['readings']
                luminance = observations.get('luminance', None)
                
                epsilon = points_width * 1.1
                points_list = []
                for theta, reading in zip(directions, readings):
                    x = np.cos(theta) * (reading - epsilon)
                    y = np.sin(theta) * (reading - epsilon)
                    points_list.append(Point(0, 0, z_sensor))
                    points_list.append(Point(x, y, z_sensor))
                
                points = []
                for theta, reading in zip(directions, readings):
                    x = np.cos(theta) * (reading - epsilon)
                    y = np.sin(theta) * (reading - epsilon)
                    points.append(Point(x, y, z_sensor))
                points_circle = []
                points_circle_radius = 1
                for theta in directions:
                    x = np.cos(theta) * points_circle_radius
                    y = np.sin(theta) * points_circle_radius
                    points_circle.append(Point(x, y, z_sensor))
                    
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = "sensor%s-ranges" % i
                marker.pose = ROS_Pose_from_SE3(world_pose)
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.points = points_list
                marker.scale.x = 0.1    
                marker.color = ColorRGBA(0, 0, 1, 1)
                self.publisher.publish(marker)
                
                if luminance is not None:
                    colors = []
                    for l in luminance:
                        colors.append(ColorRGBA(l, l, l, 1))
                    
                    marker = Marker()
                    marker.header.frame_id = frame_id
                    marker.header.stamp = stamp
                    marker.ns = "sensor%s-luminance" % i
                    marker.pose = ROS_Pose_from_SE3(world_pose)
                    marker.type = Marker.POINTS
                    marker.action = Marker.ADD
                    marker.points = points
                    marker.scale.x = points_width    
                    marker.scale.y = z_sensor_width
                    marker.color = ColorRGBA(0, 0, 1, 1)
                    marker.colors = colors
                    self.publisher.publish(marker)
                    
                    marker = Marker()
                    marker.header.frame_id = frame_id
                    marker.header.stamp = stamp
                    marker.ns = "sensor%s-luminance-circle" % i
                    marker.pose = ROS_Pose_from_SE3(world_pose)
                    marker.type = Marker.POINTS
                    marker.action = Marker.ADD
                    marker.points = points_circle
                    marker.scale.x = points_width
                    marker.scale.y = z_sensor_width
                    marker.color = ColorRGBA(0, 0, 1, 1)
                    marker.colors = colors
                    self.publisher.publish(marker)
    
    
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

        
            
