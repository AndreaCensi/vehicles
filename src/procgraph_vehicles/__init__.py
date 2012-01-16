from vehicles import __version__

'''
    Blocks useful for the visualization of Vehicles logs.
'''

#procgraph_info = {
#    # List of python packages
#    'requires':  ['ros']
#}


from procgraph import pg_add_this_package_models
pg_add_this_package_models(__file__, __package__)


from .vehiclelog_utils import *
from .map_display import *
from .cairo_map_display import *
from .yaml_log_reader import *
from .boot_log_reader import *
