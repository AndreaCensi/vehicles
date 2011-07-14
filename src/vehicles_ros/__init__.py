__version__ = '0.5'

from contracts import contract
import warnings

try:
    from ros import visualization_msgs
    from ros import sensor_msgs
    from ros import geometry_msgs
     
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker #@UnresolvedImport
    from sensor_msgs.msg import Image #@UnresolvedImport
    from geometry_msgs.msg import Point
    visualization_possible = True
except ImportError as e:
    msg = """ROS Visualization packages (visualization_msgs, sensor_msgs, tf)
not installed; not visualizing anything. 
             
Error: %s             
        """ % e
    warnings.warn(msg)
    visualization_possible = False
    
from .ros_conversions import *
from .ros_plot_vehicle import *
from .ros_plot_world import *

from .simulation import *
