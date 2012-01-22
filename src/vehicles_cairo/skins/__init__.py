from .robot_skins import *
from .sensors import *

from vehicles import VehicleSkin
import sys


def make_skin(f, name):
    class Skin(VehicleSkin):
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def draw_vehicle(self, cr, joints):
            f(cr, **self.kwargs)

        def draw(self, cr):
            f(cr, **self.kwargs)

    module = sys.modules['vehicles_cairo.skins']

    module.__dict__[name] = Skin

    #print('Added %s . %s' % (module, name))


make_skin(cairo_robot_skin_circular, 'circular')
make_skin(cairo_robot_skin_ddrive, 'ddrive')
make_skin(cairo_robot_skin_omni, 'omni')
make_skin(cairo_robot_skin_brai, 'brai')
make_skin(cairo_robot_skin_car, 'car')

make_skin(cairo_skin_eye, 'eye')
make_skin(cairo_skin_sick, 'sick')
make_skin(cairo_skin_transparent, 'transparent')
