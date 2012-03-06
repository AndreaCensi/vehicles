from .. import np, logger

_multiprocess_can_split_ = True # Run parallel tests

from .instantiation import *
from .generation import *

from . import display

from  . import simple_raytracer_test
from  . import simulation_tests
from  . import test_instance_all
