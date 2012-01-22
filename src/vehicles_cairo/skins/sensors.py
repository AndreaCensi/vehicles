from .. import cairo_plot_rectangle, cairo_plot_circle2

WHITE = [1, 1, 1]
BLACK = [0, 0, 0]
YELLOW = [1, 1, 0]


def cairo_skin_sick(cr, w=0.25):
    cairo_plot_rectangle(cr, -w / 2, -w / 2, w, w,
                         fill_color=YELLOW,
                         border_color=BLACK,
                         border_width=0.005)

    w2 = w / 1.6
    l = w / 3
    cairo_plot_rectangle(cr, w / 2 - l, -w2 / 2, l, w2,
                         fill_color=BLACK)


def cairo_skin_eye(cr, w=0.25):
    # White part
    cairo_plot_circle2(cr, 0, 0, 0.2,
                       fill_color=WHITE, border_color=BLACK,
                       border_width=0.005)

    # Brown
    cairo_plot_circle2(cr, 0.1, 0, 0.1,
                       fill_color=[.4, .2, 0],
                       border_color=None,
                       border_width=None)

    # Black
    cairo_plot_circle2(cr, 0.12, 0, 0.05,
                       fill_color=BLACK,
                       border_color=None,
                       border_width=None)


def cairo_skin_transparent(cr):
    pass
