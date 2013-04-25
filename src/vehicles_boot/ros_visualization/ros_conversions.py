from contracts import contract
from geometry import quaternion_from_rotation, rotation_translation_from_pose
import numpy as np


@contract(pose='SE3')
def ROS_Pose_from_SE3(pose, z=0):
    ''' Converts an element of SE3 (as a 4x4 matrix) into the ROS format. '''
    from geometry_msgs.msg import Pose, Point  # @UnresolvedImport
    R, t = rotation_translation_from_pose(pose)
    q1 = quaternion_from_rotation(R)
    q = ROS_quaternion_from_my_quaternion(q1)
    return Pose(Point(t[0], t[1], t[2]), q)


def ROS_quaternion_order(q):
    return np.array([q[1], q[2], q[3], q[0]])


def ROS_quaternion_from_my_quaternion(q):
    from geometry_msgs.msg import Quaternion  # @UnresolvedImport
    q = ROS_quaternion_order(q)
    return Quaternion(q[0], q[1], q[2], q[3])

# def test_same_convention():
#    """ Make sure that ROS and I agree on the conventions for rotations. """
#    import tf #@UnresolvedImport
#    import rospy #@UnresolvedImport
#    theta = 0.2
#    q1 = quaternion_from_axis_angle(axis=np.array([0, 0, 1]), angle=theta)
#    q1f = ROS_quaternion_from_my_quaternion(q1)
#    q2 = tf.transformations.quaternion_from_euler(0, 0, theta)
#    rospy.loginfo('q1f: %s' % q1f)
#    rospy.loginfo('q2 : %s' % q2)


# found somewhere
def numpy_to_imgmsg(image, stamp=None):
    from sensor_msgs.msg import Image 
    rosimage = Image()
    rosimage.height = image.shape[0]
    rosimage.width = image.shape[1]
    if image.dtype == np.uint8:
        rosimage.encoding = '8UC%d' % image.shape[2]
        rosimage.step = image.shape[2] * rosimage.width
        rosimage.data = image.ravel().tolist()
    else:
        rosimage.encoding = '32FC%d' % image.shape[2]
        rosimage.step = image.shape[2] * rosimage.width * 4
        rosimage.data = np.array(image.flat, dtype=np.float32).tostring()
    if stamp is not None:
        rosimage.header.stamp = stamp
    return rosimage


