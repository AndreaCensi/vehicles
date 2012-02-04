
from . import np
from cairo_utils import (cairo_save, cairo_set_color, cairo_transform,
    cairo_plot_rectangle, cairo_plot_circle2)

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
        #BROWN1 = [.4, .2, 0]
        BROWN2 = [.6, .3, 0]
        cairo_plot_circle2(cr, 0.5, 0, 0.5,
                           fill_color=BROWN2,
                           border_color=None,
                           border_width=None)

        # Black
        cairo_plot_circle2(cr, 0.6, 0, 0.22, #0.25,
                           fill_color=BLACK,
                           border_color=None,
                           border_width=None)


def cairo_skin_transparent(cr):
    pass


def cairo_ref_frame(cr, l=1, x_color=[1, 0, 0], y_color=[0, 1, 0]):
    with cairo_save(cr):
        cairo_arrow(cr, length=l, border_width=0.05, border_color=x_color)
        with cairo_transform(cr, r=np.pi / 2):
            cairo_arrow(cr, length=l, border_width=0.05, border_color=y_color)


def cairo_arrow(cr, length, caplen=0.05, capwidth=0.05,
                border_width=0.05, border_color=[0, 0, 0]):

    cr.set_line_width(border_width)
    cairo_set_color(cr, border_color)

    cr.move_to(0, 0)
    cr.line_to(length, 0)
    cr.stroke()

    cr.move_to(length - caplen, capwidth / 2.0)
    cr.line_to(length, 0)
    cr.line_to(length - caplen, -capwidth / 2.0)
    cr.fill()
