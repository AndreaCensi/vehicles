''' PyVehicles main package: play with your vehicles zoo! '''

__version__ = '2.0'

__all__ = ['VehicleSensor', 'Field', 'PolyLine', 'Vehicle', 'Circle',
           'GeometricShape', 'VehiclesConstants', 'VehicleSimulation',
            'VehiclesConfig', 'World',
             'Attached', 'Primitive', 'Source', 'VehicleSkin', 'Dynamics',
             'library']

from contracts import contract
import geometry  # loads all geometry contracts
import numpy as np

import logging
logging.basicConfig()
logger = logging.getLogger("Vehicles")
logger.setLevel(logging.DEBUG)

from .constants import *
from .configuration import *
from .interfaces import *
from .simulation import *

from . import library

__docformat__ = 'restructuredtext' # For Epydoc
