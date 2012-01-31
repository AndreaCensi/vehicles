''' Plotting using the Cairo library. '''

from vehicles import __version__, logger


class CairoConstants:
    BLACK = [0, 0, 0]
    WHITE = [1, 1, 1]
    YELLOW = [1, 1, 0]

    obstacle_border_width = 0.01
    obstacle_border_color = [0, 0, 0]
    obstacle_fill_color = [0.7, 0.7, 0.8]

    omni_wheel_style = dict(border_color=BLACK, border_width=0.01,
                            fill_color=[.3, .3, 1])

    robot_wheel_style = dict(border_color=BLACK, border_width=0.04,
                            fill_color=[.2, .2, .2])

    grid_color = [0.8, 0.8, 0.8]
    grid_width = 0.005

    robot_body_style = dict(border_width=0.01, border_color=BLACK,
                          fill_color=[0.8, 0.8, 0.8])

    robot_border_width = 0.01
    robot_border_color = BLACK
    robot_fill_color = [0.8, 0.8, 0.8]

    robot_wheel_fill_color = [.2, .2, .2]
    robot_wheel_border_width = 0.04
    robot_wheel_border_color = BLACK


    texture_resolution = 0.05 # meters
    texture_width = 0.2 # meters

try:
    import cairo #@UnresolvedImport
    vehicles_has_cairo = True

except ImportError as e:
    cairo_error = e
    vehicles_has_cairo = False

else:
    from .cairo_utils import *
    from .utils import *
    from .skins import *
    from .world_geometry import *
    from .sensor_data import *
    from .display_all import *

