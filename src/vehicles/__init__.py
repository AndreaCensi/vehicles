__version__ = '0.5'
from contracts import contract
import geometry # loads all geometry contracts
import numpy as np
import logging

logging.basicConfig();
logger = logging.getLogger("vehicles")
logger.setLevel(logging.DEBUG)


from .configuration import *
from .interfaces import *
from .simulation import *
from .sensors import *
from .worlds import *
