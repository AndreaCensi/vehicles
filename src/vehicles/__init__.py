__version__ = '1.0'

from contracts import contract
import geometry # loads all geometry contracts
import numpy as np
import logging

logging.basicConfig();
logger = logging.getLogger("Vehicles")
logger.setLevel(logging.DEBUG)


from .configuration import *
from .interfaces import *
from .simulation import *
from .sensors import *
from .worlds import *

