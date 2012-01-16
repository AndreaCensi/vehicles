

class CairoConstants:
    obstacle_border_width = 0.01
    obstacle_border_color = [0.7, 0.7, 0.8]
    obstacle_fill_color = [0.7, 0.7, 0.8]
    grid_color = [0, 0, 0]
    grid_width = 0.005


try:
    import cairo #@UnresolvedImport
    vehicles_has_cairo = True

except ImportError as e:
    cairo_error = e
    vehicles_has_cairo = False

else:
    from .utils import *
    from .world_geometry import *
    from .sensor_data import *
    from .display_all import *

