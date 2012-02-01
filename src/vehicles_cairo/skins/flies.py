from . import np
from .. import BLACK, WHITE
from cairo_utils import cairo_plot_circle2, cairo_save, cairo_transform


def cairo_skin_fly1(cr):

    r = 1.0 # will be scaled by robot radius

    with cairo_save(cr):
        cr.scale(0.32, 0.32)

        def wing():
            rw = r
            with cairo_save(cr):
                cr.scale(1.5, 1)
                cairo_plot_circle2(cr, rw / 2, 0, rw,
                           fill_color=[.5, .5, .5])

        wa = 3 / 2
        with cairo_transform(cr, t=[0, wa], r=np.pi / 2):
            wing()

        with cairo_transform(cr, t=[0, -wa], r=(-np.pi / 2)):
            wing()

        with cairo_save(cr):
            cr.scale(1.5, 0.75)
            cairo_plot_circle2(cr, 0, 0, r,
                       fill_color=WHITE, border_color=BLACK,
                       border_width=0.005)

        cairo_plot_circle2(cr, r * 1.5, 0, r / 1.5,
                       fill_color=WHITE, border_color=BLACK,
                       border_width=0.005)
