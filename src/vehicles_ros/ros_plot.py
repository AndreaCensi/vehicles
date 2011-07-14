
try:
    from ros import visualization_msgs
    from ros import sensor_msgs
    from ros import geometry_msgs
     
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker #@UnresolvedImport
    from sensor_msgs.msg import Image #@UnresolvedImport

    from geometry_msgs.msg import Point
except ImportError as e:
    pass


def get_marker_for_world_extent(bounds, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "extent"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
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
