'''
    Blocks for the visualization of Vehicles logs using Procgraph.
'''

from vehicles import __version__, logger
from vehicles_cairo import vehicles_has_cairo

# procgraph_info = {
#    # List of python packages
#    'requires':  ['ros']
# }


from procgraph import pg_add_this_package_models
pg_add_this_package_models(__file__, __package__)

if vehicles_has_cairo:
    from .cairo_map_display import *
else:
    logger.warn('No Cairo support available - cairo_map_display not loaded')
    
from .vehiclelog_utils import *
from .yaml_log_reader import *

