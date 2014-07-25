from . import Marker, ColorRGBA, Point, ROS_Pose_from_SE3
from geometry import SE3_from_rotation_translation
import numpy as np
from vehicles.library.sensors import MyRaytracer


def publish_vehicle(publisher, params, vehicle):
    frame_id = params['world_frame']
    stamp = params['stamp']
    robot_height = params['robot_height']

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "vehicle"
    marker.id = 0
    marker.type = Marker.CUBE  # @UndefinedVariable
    marker.action = Marker.ADD       # @UndefinedVariable
    pose_marker = SE3_from_rotation_translation(
                            np.eye(3),
                            np.array([0, 0, robot_height / 2]))
    vehicle_pose = vehicle.get_pose()
    marker.pose = ROS_Pose_from_SE3(np.dot(vehicle_pose, pose_marker))
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = robot_height
    marker.color = ColorRGBA(1.0, 0.3, 0.3, 1.0)
    publisher.publish(marker)

    if params['visualize_sensors']:
        for i, attached in enumerate(vehicle.sensors):
            sensor = attached.sensor
            if isinstance(sensor, MyRaytracer): # XXX
                visualize_raytracer(publisher, params, i, attached)


def visualize_raytracer(publisher, params, i, attached):
    points_width = params['points_width']
    z_sensor_width = params['z_sensor_width']
    stamp = params['stamp']
    frame_id = params['world_frame']
    z_sensor = params['z_sensor']
    epsilon = params['points_width'] * 1.5

    sensor = attached.sensor
    observations = attached.current_observations
    world_pose = attached.current_pose
    directions = sensor.directions
    readings = observations['readings']
    luminance = observations.get('luminance', None)

    points_list = []
    for theta, reading in zip(directions, readings):
        r0 = 0.5
        x0 = np.cos(theta) * r0
        y0 = np.sin(theta) * r0
        points_list.append(Point(x0, y0, z_sensor))
        x = np.cos(theta) * (reading - epsilon)
        y = np.sin(theta) * (reading - epsilon)
        points_list.append(Point(x, y, params['z_sensor']))

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
    marker.type = Marker.LINE_LIST  # @UndefinedVariable
    marker.action = Marker.ADD  # @UndefinedVariable
    marker.points = points_list
    marker.scale.x = 0.03
    marker.color = ColorRGBA(0, 0, 1, 1)
    publisher.publish(marker)

    if luminance is not None:
        colors = []
        for l in luminance:
            colors.append(ColorRGBA(l, l, l, 1))

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "sensor%s-luminance" % i
        marker.pose = ROS_Pose_from_SE3(world_pose)
        marker.type = Marker.POINTS  # @UndefinedVariable
        marker.action = Marker.ADD  # @UndefinedVariable
        marker.points = points
        marker.scale.x = points_width
        marker.scale.y = z_sensor_width
        marker.color = ColorRGBA(0, 0, 1, 1)
        marker.colors = colors
        publisher.publish(marker)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "sensor%s-luminance-circle" % i
        marker.pose = ROS_Pose_from_SE3(world_pose)
        marker.type = Marker.POINTS  # @UndefinedVariable
        marker.action = Marker.ADD  # @UndefinedVariable
        marker.points = points_circle
        marker.scale.x = points_width
        marker.scale.y = z_sensor_width
        marker.color = ColorRGBA(0, 0, 1, 1)
        marker.colors = colors
        publisher.publish(marker)


