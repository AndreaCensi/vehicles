from contextlib import contextmanager
from contracts import contract
import geometry
from cairo_utils import cairo_transform


@contract(a='seq[4](number)')
def cairo_set_axis(cr, width, height, a):
    xmin, xmax, ymin, ymax = a
    cr.translate(width / 2, height / 2)
    Z = max(float(xmax - xmin) / width,
            float(ymax - ymin) / height)
    assert Z > 0
    cr.scale(1 / Z, -1 / Z)
    xmid = (xmin + xmax) / 2.0
    ymid = (ymin + ymax) / 2.0
    cr.translate(-xmid, -ymid)


@contextmanager
@contract(pose='SE2')
def cairo_rototranslate(cr, pose):
    t, r = geometry.translation_angle_from_SE2(pose)
    with cairo_transform(cr, t=t, r=r)as cr:
        yield cr


