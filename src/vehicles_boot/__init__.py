''' Code interfacing PyVehicles with BootstrappingOlympics. '''

from vehicles import __version__, logger

from contracts import contract
import numpy as np

try:
    import bootstrapping_olympics
    vehicles_has_boot_olympics = True
except ImportError:
    logger.error('Package BootOlympics not installed, vehicles_boot cannot '
                 'work properly.')
    vehicles_has_boot_olympics = False

if vehicles_has_boot_olympics:
    from .vehicles_simulation import *
    from .ros_visualization import *

