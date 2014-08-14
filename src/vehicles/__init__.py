''' PyVehicles main package: play with your vehicles zoo! '''

__version__ = '2.1'
 
__all__ = ['VehicleSensor', 'Field', 'PolyLine', 'Vehicle', 'Circle',
           'GeometricShape', 'VehiclesConstants', 'VehicleSimulation',
             'World', 'Attached', 'Primitive', 'Source', 'VehicleSkin', 'Dynamics',
             'library']

# Does extra checks to make sure states, etc. belong to their manifolds.
# These are now redundant, but it was useful while debugging.
# Reactivate if some strange bug is suspected.
DO_EXTRA_CHECKS = False


import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

import geometry  # loads all geometry contracts

from .constants import *
from .configuration import *
from .interfaces import *
from .simulation import *

from . import library

__docformat__ = 'restructuredtext'  # For Epydoc
 

def jobs_comptests(context):
    from conf_tools import GlobalConfig
    GlobalConfig.global_load_dirs(['vehicles.configs'])

    from . import unittests

    from comptests import jobs_registrar
    jobs_registrar(context, get_vehicles_config())
    