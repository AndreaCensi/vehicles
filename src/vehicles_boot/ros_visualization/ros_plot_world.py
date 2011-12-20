from . import Marker, ColorRGBA, Point, ROS_Pose_from_SE3
from geometry import SE3_from_rotation_translation
from vehicles import Circle, PolyLine
import numpy as np


def publish_world(publisher, params, world):
    frame_id = params['world_frame']
    stamp = params['stamp']
    z0 = params['z0']
    z1 = params['z1']

    marker = get_marker_for_world_extent(world.bounds, frame_id, stamp)
    publisher.publish(marker)

    # TODO: only updated
    for x in world.get_primitives():
        surface = x.id_object
        if isinstance(x, PolyLine):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "world"
            marker.id = surface
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD  # @UndefinedVariable
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
            publisher.publish(marker)

        elif isinstance(x, Circle):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "world"
            marker.id = surface
            marker.type = Marker.CYLINDER  # @UndefinedVariable
            marker.action = Marker.ADD  # @UndefinedVariable
#            marker.points = [Point(x.center[0], x.center[1], 
#                                0.5 * z0 + 0.5 * z1)]
            marker.scale.x = x.radius * 2
            marker.scale.y = x.radius * 2
            z = 0.5 * z0 + 0.5 * z1
            translation = np.array([x.center[0], x.center[1], z])
            pose = SE3_from_rotation_translation(np.eye(3), translation)
            marker.pose = ROS_Pose_from_SE3(pose)
            marker.scale.z = (z1 - z0)
            marker.color = ColorRGBA(.5, .5, .2, 1)
            publisher.publish(marker)
        else:
            raise Exception('Unexpected object %s' % x)


def get_marker_for_world_extent(bounds, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "extent"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST  # @UndefinedVariable
    marker.action = Marker.ADD  # @UndefinedVariable
    vertices = []

    a = Point(bounds[0][0], bounds[1][0], 0)
    b = Point(bounds[0][0], bounds[1][1], 0)
    c = Point(bounds[0][1], bounds[1][1], 0)
    d = Point(bounds[0][1], bounds[1][0], 0)
    vertices.extend([a, b, c])
    vertices.extend([c, d, a])
    marker.points = vertices
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color = ColorRGBA(0, 0.4, 0, 1)
    return marker
