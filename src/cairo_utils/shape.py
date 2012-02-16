from . import cairo_save, cairo_set_color, np


def cairo_plot_polyline(cr, x, y):
    cr.move_to(x[0], y[0])
    for i in range(1, len(x)):
        cr.line_to(x[i], y[i])
    cr.stroke()


# TODO: add deprecations
#@contract(center='seq[2](float)', radius='>0')
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


def cairo_plot_with_style(cr, shape, border_color=None,
                         border_width=None,
                         fill_color=None):
    with cairo_save(cr):

        if fill_color is not None:
            shape()
            cairo_set_color(cr, fill_color)
            cr.fill()

        # TODO: fill_preserve
        if border_color is not None:
            if border_width is not None:
                cr.set_line_width(border_width)
            shape()
            cairo_set_color(cr, border_color)
            cr.stroke()


def cairo_stroke_with_style(cr, shape, border_color=None,
                                       border_width=None,
                                       fill_color=None):
    with cairo_save(cr):

        if border_color is not None:
            if border_width is not None:
                cr.set_line_width(border_width)
            shape()
            cairo_set_color(cr, border_color)
            cr.stroke()


def cairo_plot_rectangle(cr, x, y, w, h, **kwargs):
    def shape():
        cr.rectangle(x, y, w, h)

    cairo_plot_with_style(cr, shape, **kwargs)


def cairo_plot_circle2(cr, x, y, radius, **kwargs):
    def shape():
        cr.arc(x, y, radius, 0, 2 * np.pi)

    cairo_plot_with_style(cr, shape, **kwargs)


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
