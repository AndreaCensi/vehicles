from contextlib import contextmanager
import numpy as np

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


def cairo_plot_circle(cr, center, radius, edgecolor=None,
                      facecolor=None, width=None):
    with cairo_save(cr):
        cr.translate(center[0], center[1])
        cr.scale(radius, radius)

        if facecolor is not None:
            cr.arc(0, 0, 1, 0, 2 * np.pi)
            cr.set_source_rgb(*facecolor)
            cr.fill()

        if width is not None:
            cr.set_line_width(width)

        if edgecolor is not None:
            cr.arc(0, 0, 1, 0, 2 * np.pi)
            cr.set_source_rgb(*edgecolor)
            cr.stroke()

