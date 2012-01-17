from contextlib import contextmanager
import numpy as np
from contracts import contract
import geometry


def cairo_plot_polyline(cr, x, y):
    cr.move_to(x[0], y[0])
    for i in range(1, len(x)):
        cr.line_to(x[i], y[i])
    cr.stroke()


@contextmanager
def cairo_save(cr):
    cr.save()
    yield cr
    cr.restore()


@contextmanager
def cairo_transform(cr, t=[0, 0], r=0, s=1):
    with cairo_save(cr):
        cr.translate(t[0], t[1])
        cr.rotate(r)
        cr.scale(s, s)
        yield cr


@contextmanager
@contract(pose='SE2')
def cairo_rototranslate(cr, pose):
    t, r = geometry.translation_angle_from_SE2(pose)
    with cairo_transform(cr, t=t, r=r)as cr:
        yield cr


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


@contract(col='seq[3]|seq[4]')
def cairo_set_color(cr, col):
    if len(col) == 3:
        cr.set_source_rgb(col[0], col[1], col[2])
    else:
        cr.set_source_rgba(col[0], col[1], col[2], col[3])


def cairo_plot_circle(cr, center, radius, edgecolor=None,
                      facecolor=None, width=None):
    with cairo_save(cr):
        cr.translate(center[0], center[1])
        cr.scale(radius, radius)

        if facecolor is not None:
            cr.arc(0, 0, 1, 0, 2 * np.pi)
            cairo_set_color(cr, facecolor)
            cr.fill()

        if width is not None:
            cr.set_line_width(width)

        if edgecolor is not None:
            cr.arc(0, 0, 1, 0, 2 * np.pi)
            cairo_set_color(cr, edgecolor)
            cr.stroke()


def roundedrec(context, x, y, w, h, r=10):
    "Draw a rounded rectangle"
    #   A****BQ
    #  H      C
    #  *      *
    #  G      D
    #   F****E

    context.move_to(x + r, y)
    context.line_to(x + w - r, y)
    context.curve_to(x + w, y, x + w, y, x + w, y + r)
    context.line_to(x + w, y + h - r)
    context.curve_to(x + w, y + h, x + w, y + h, x + w - r, y + h)
    context.line_to(x + r, y + h)
    context.curve_to(x, y + h, x, y + h, x, y + h - r)
    context.line_to(x, y + r)
    context.curve_to(x, y, x, y, x + r, y)

