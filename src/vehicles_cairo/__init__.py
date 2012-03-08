''' 
    Plotting Vehicles using the Cairo library. 
    
    Use this pattern: ::
    
        # Check for Cairo support
        from vehicles_cairo import vehicles_has_cairo
        if not vehicles_has_cairo:
            logger.error('This program cannot run if Cairo is not installed.')
            return
    
        from vehicles_cairo import ...

'''

from vehicles import __version__, logger
import numpy as np
from contracts import contract


try:
    import cairo #@UnresolvedImport
    vehicles_has_cairo = True

except ImportError as e:
    cairo_error = e
    vehicles_has_cairo = False
    logger.warning('Could not import PyCairo; visualization not available.')
    
else:
    from .constants import *
    from .utils import *
    from .skins import *
    from .world_geometry import *
    from .sensor_data import *
    from .display_all import *
    from .write_to_file import *

