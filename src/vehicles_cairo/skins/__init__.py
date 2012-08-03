from vehicles import VehicleSkin

from .. import np, contract

from .robot_skins import *
from .sensors import *
from .flies import *
from .complex_skin import *

import sys


def make_skin(f, name):
    class Skin(VehicleSkin):
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def draw(self, cr, joints=None, timestamp=None): #@UnusedVariable
            f(cr, **self.kwargs)

    module = sys.modules['vehicles_cairo.skins']

    module.__dict__[name] = Skin

    #print('Added %s . %s' % (module, name))


make_skin(cairo_robot_skin_circular, 'circular')
make_skin(cairo_robot_skin_ddrive, 'ddrive')
make_skin(cairo_robot_skin_omni, 'omni')
make_skin(cairo_robot_skin_brai, 'brai')
make_skin(cairo_robot_skin_car, 'car')
make_skin(cairo_robot_skin_tracked, 'tracks')


make_skin(wheel, 'wheel')


make_skin(cairo_skin_eye, 'eye')
make_skin(cairo_skin_sick, 'sick')
make_skin(cairo_skin_transparent, 'transparent')

make_skin(cairo_skin_fly1, 'fly1')

make_skin(cairo_robot_skin_rectangle, 'rectangle')
make_skin(cairo_robot_skin_roundedrec, 'roundedrec')

