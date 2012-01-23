from .. import cairo_plot_rectangle, cairo_plot_circle2
from vehicles_cairo.cairo_utils.contexts import cairo_save

WHITE = [1, 1, 1]
BLACK = [0, 0, 0]
YELLOW = [1, 1, 0]


def cairo_skin_sick(cr, size=1.0):
    with cairo_save(cr):
        cr.scale(size, size)
        cairo_plot_rectangle(cr, -.5, -.5, 1, 1,
                             fill_color=YELLOW,
                             border_color=BLACK,
                             border_width=0.005)

        w2 = 1.0 / 1.6
        l = 1.0 / 3
        cairo_plot_rectangle(cr, .5 - l, -w2 / 2, l, w2,
                             fill_color=BLACK)


def cairo_skin_eye(cr, size=1.0):
    with cairo_save(cr):
        cr.scale(size, size)
        # White part
        cairo_plot_circle2(cr, 0, 0, 1,
                           fill_color=WHITE, border_color=BLACK,
                           border_width=0.05)

        # Brown
        cairo_plot_circle2(cr, 0.5, 0, 0.5,
                           fill_color=[.4, .2, 0],
                           border_color=None,
                           border_width=None)

        # Black
        cairo_plot_circle2(cr, 0.6, 0, 0.25,
                           fill_color=BLACK,
                           border_color=None,
                           border_width=None)


def cairo_skin_transparent(cr):
    pass
