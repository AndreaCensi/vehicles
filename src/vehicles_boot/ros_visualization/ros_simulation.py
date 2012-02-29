from . import publish_vehicle, numpy_to_imgmsg
from .. import BOVehicleSimulation
from contracts import contract
import contracts
import numpy as np
import rospy # @UnresolvedImport
import yaml
from vehicles_boot.ros_visualization.ros_plot_world import publish_world


class VizLevel:
    # Visualization levels
    Nothing = 0
    State = 0  # TODO: reconsider
    Geometry = 1
    Sensels = 2
    SensorData = 3
    Everything = 3


class ROSVehicleSimulation(BOVehicleSimulation):

    def __init__(self, **params):
        contracts.disable_all()  # XXX

        self.viz_level = params.get('viz_level', VizLevel.Everything)

        if self.viz_level > VizLevel.Nothing:
            from . import Marker, Image
            self.publisher = rospy.Publisher('~markers', Marker)
            self.pub_sensels_image = rospy.Publisher('~sensels_image', Image)
            self.pub_commands_image = rospy.Publisher('~commands_image', Image)
            self.first_time = True

        if self.viz_level >= VizLevel.State:
            from . import  String
            self.pub_state = rospy.Publisher('~state', String)

        BOVehicleSimulation.__init__(self, **params)

    def info(self, s):
        rospy.loginfo(s)

    def set_commands(self, commands):
        BOVehicleSimulation.set_commands(self, commands)

        if self.viz_level >= VizLevel.Sensels:
            self.publish_ros_commands(commands)
        if self.viz_level >= VizLevel.Geometry:
            self.publish_ros_markers()

        if self.vehicle_collided:  # FIXME:
            rospy.loginfo('Restarting new episode due to collision.')
            self.new_episode()

    def get_observations(self):
        timestamp, observations = BOVehicleSimulation.get_observations(self)

        if self.viz_level >= VizLevel.Sensels:
            self.publish_ros_sensels(observations)

        if self.viz_level >= VizLevel.State:
            from . import String
            y = self.to_yaml()
            s = yaml.dump(y)
            self.pub_state.publish(String(s))

        return timestamp, observations

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

