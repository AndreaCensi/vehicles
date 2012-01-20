''' Plotting using the Cairo library. '''


class CairoConstants:
    BLACK = [0, 0, 0]

    obstacle_border_width = 0.01
    obstacle_border_color = [0, 0, 0]
    obstacle_fill_color = [0.7, 0.7, 0.8]

    grid_color = [0.5, 0.5, 0.5]
    grid_width = 0.005

    robot_border_width = 0.01
    robot_border_color = BLACK
    robot_fill_color = [0.8, 0.8, 0.8]

    robot_wheel_fill_color = [.2, .2, .2]
    robot_wheel_border_width = 0.04
    robot_wheel_border_color = BLACK


try:
    import cairo #@UnresolvedImport
    vehicles_has_cairo = True

except ImportError as e:
    cairo_error = e
    vehicles_has_cairo = False

else:
    from .utils import *
    from .skins import *
    from .world_geometry import *
    from .sensor_data import *
    from .display_all import *

