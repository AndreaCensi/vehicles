from .. import BLACK, CairoConstants, np
from cairo_utils import (cairo_plot_rectangle, cairo_plot_circle2,
    cairo_plot_with_style, cairo_stroke_with_style, roundedrec, cairo_save,
    cairo_set_color, cairo_transform)


def cairo_robot_skin_circular(cr):
    with cairo_save(cr):
        cairo_plot_circle2(cr, 0, 0, 1, **CairoConstants.robot_body_style)

        def line():
            cr.move_to(0.2, 0)
            cr.line_to(1, 0)

        cairo_stroke_with_style(cr, line, **CairoConstants.robot_body_style)


def cairo_robot_skin_rectangle(cr, w, h):
    """ A simple rectangle painted like the robot. """
    with cairo_save(cr):
        cairo_plot_rectangle(cr, -w / 2.0, -h / 2.0, w, h,
                             ** CairoConstants.robot_body_style)


# TODO: param color
def cairo_robot_skin_roundedrec(cr, w, h, r=None):
    if r is None:
        r = min(w, h) / 3.0
    with cairo_save(cr):
        def shape():
            roundedrec(cr, -w / 2.0, -h / 2.0, w, h, r=r)
        cairo_plot_with_style(cr, shape, **CairoConstants.robot_body_style)


def cairo_robot_skin_tracked(cr, width=1.0, length=1.0):
    with cairo_save(cr):
        def track():
            wheel(cr, w=length * .7, h=width / 3.0)

        with cairo_transform(cr, t=[0, -width / 2.0]):
            track()

        with cairo_transform(cr, t=[0, +width / 2.0]):
            track()

        cairo_plot_rectangle(cr, -length / 2.0, -width / 2.0,
                             length, width,
                             **CairoConstants.robot_body_style)


def wheel(cr, w=0.4, h=0.15):
    with cairo_save(cr):
        def shape():
            roundedrec(cr, -w / 2.0, -h / 2.0, w, h, r=min(w, h) / 3.0)

        cairo_plot_with_style(cr, shape, **CairoConstants.robot_wheel_style)


def omni_wheel(cr, r=0.15):
    cairo_plot_circle2(cr, 0, 0, r, **CairoConstants.omni_wheel_style)


def cairo_robot_skin_ddrive(cr):
    with cairo_save(cr):

        with cairo_transform(cr, [0, +.8], 0):
            wheel(cr)

        with cairo_transform(cr, [0, -.8], 0):
            wheel(cr)

        cr.scale(0.7, 0.7) # make it stay in < 1
        cairo_robot_skin_circular(cr)


def cairo_robot_skin_omni(cr):
    with cairo_save(cr):

        for theta in [0, 2 * np.pi / 3, -2 * np.pi / 3]:
            with cairo_transform(cr, r=theta - np.pi / 2):
                with cairo_transform(cr, [0, +.8], np.pi / 2):
                    omni_wheel(cr)

        cr.scale(0.7, 0.7) # make it stay in < 1
        cairo_robot_skin_circular(cr)


def cairo_robot_skin_brai(cr, w=1.3, h=0.8, sensors=False):
    with cairo_save(cr):
        x0 = -.3
        wheel_h = 0.15
        wheel_w = 0.4
        M = h / 2 + .8 * wheel_h

        def body():
            roundedrec(cr, x0, -h / 2, w, h, r=min(w, h) / 8.0)
            #cr.rectangle(x0, -h / 2, w, h)

        cr.move_to(0, M)
        cr.line_to(0, -M)
        cairo_set_color(cr, BLACK)
        cr.set_line_width(0.05)
        cr.stroke()

        cairo_plot_with_style(cr, body, **CairoConstants.robot_body_style)

        for y in [-M, +M]:
            with cairo_transform(cr, t=[0, y]):
                wheel(cr, w=wheel_w, h=wheel_h)

        if sensors:
            for y in [.3 * h, -.3 * h]:
                with cairo_transform(cr, t=[0, y]):
                    cr.set_line_width(CairoConstants.robot_border_width)
                    cairo_set_color(cr, CairoConstants.robot_border_color)
                    cr.move_to(0, 0)
                    cr.line_to(1.1, 0)
                    cr.stroke()
                    r = .1
                    with cairo_transform(cr, t=[1.1 + r, 0]):
                        cr.arc(0, 0, r, +np.pi / 2, -np.pi / 2)
                        cr.stroke()


def cairo_robot_skin_car(cr):
    cairo_robot_skin_brai(cr, sensors=False)

