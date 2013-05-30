from vehicles import VehicleSkin


from .robot_skins import *
from .sensors import *
from .flies import *
from .complex_skin import *

import sys


def make_skin(f, name):
    class Skin(VehicleSkin):
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def draw(self, cr, joints=None, timestamp=None):  # @UnusedVariable
            f(cr, **self.kwargs)

    # module = sys.modules['vehicles_cairo.skins']
    Skin.__name__ = name
    # module.__dict__[name] = Skin
    return Skin
    # print('Added %s . %s' % (module, name))


circular = make_skin(cairo_robot_skin_circular, 'circular')
ddrive = make_skin(cairo_robot_skin_ddrive, 'ddrive')
omni = make_skin(cairo_robot_skin_omni, 'omni')
brai = make_skin(cairo_robot_skin_brai, 'brai')
car = make_skin(cairo_robot_skin_car, 'car')
tracks = make_skin(cairo_robot_skin_tracked, 'tracks')


wheel = make_skin(wheel, 'wheel')


eye = make_skin(cairo_skin_eye, 'eye')
sick = make_skin(cairo_skin_sick, 'sick')
transparent = make_skin(cairo_skin_transparent, 'transparent')

fly1 = make_skin(cairo_skin_fly1, 'fly1')

rectangle = make_skin(cairo_robot_skin_rectangle, 'rectangle')
roundedrec = make_skin(cairo_robot_skin_roundedrec, 'roundedrec')

