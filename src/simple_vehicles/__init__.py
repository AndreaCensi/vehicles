import logging

logging.basicConfig();
logger = logging.getLogger("vehicles")
logger.setLevel(logging.DEBUG)


from .loading import *
from .olympics import *
