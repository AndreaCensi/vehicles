from . import cairo_transform, cairo_save, cairo_set_color, CairoConstants, np


def roundedrec(context, x, y, w, h, r=10):
    "Draw a rounded rectangle"
    #   A****BQ
    #  H      C
    #  *      *
    #  G      D
    #   F****E

    context.move_to(x + r, y)                      # Move to A
    context.line_to(x + w - r, y)                    # Straight line to B
    context.curve_to(x + w, y, x + w, y, x + w, y + r)       # Curve to C, Control points are both at Q
    context.line_to(x + w, y + h - r)                  # Move to D
    context.curve_to(x + w, y + h, x + w, y + h, x + w - r, y + h) # Curve to E
    context.line_to(x + r, y + h)                    # Line to F
    context.curve_to(x, y + h, x, y + h, x, y + h - r)       # Curve to G
    context.line_to(x, y + r)                      # Line to H
    context.curve_to(x, y, x, y, x + r, y)             # Curve to A



def cairo_robot_skin_circular(cr):
    with cairo_save(cr):
        cr.set_line_width(CairoConstants.robot_border_width)
        cairo_set_color(cr, CairoConstants.robot_fill_color)
        cr.arc(0, 0, 1, 0, 2 * np.pi)
        cr.fill()
        cr.arc(0, 0, 1, 0, 2 * np.pi)
        cairo_set_color(cr, CairoConstants.robot_border_color)
        cr.stroke()
        cr.move_to(0.2, 0)
        cr.line_to(1, 0)
        cr.stroke()


def wheel(cr, w=0.7, h=0.5):
    with cairo_save(cr):
        def shape():
#            cr.rectangle(-w / 2, -h / 2, w, h)
            roundedrec(cr, -w / 2, -h / 2, w, h, r=w / 3)
        shape()
        cairo_set_color(cr, CairoConstants.robot_wheel_fill_color)
        cr.fill()
        shape()
        cr.set_line_width(CairoConstants.robot_wheel_border_width)
        cairo_set_color(cr, CairoConstants.robot_wheel_border_color)
        cr.stroke()


def omni_wheel(cr):
    wheel(cr) # TODO: make different color


def cairo_robot_skin_ddrive(cr):
    with cairo_save(cr):

        cr.scale(0.8, 0.8) # make it stay in <1 

        with cairo_transform(cr, [0, +.9], 0):
            wheel(cr)

        with cairo_transform(cr, [0, -.9], 0):
            wheel(cr)

        cairo_robot_skin_circular(cr)


def cairo_robot_skin_omni(cr):
    with cairo_save(cr):

        cr.scale(0.8, 0.8) # make it stay in < 1

        for theta in [0, 2 * np.pi / 3, -2 * np.pi / 3]:
            with cairo_transform(cr, r=theta - np.pi / 2):
                with cairo_transform(cr, [0, +.9], np.pi / 2):
                    omni_wheel(cr)

        cairo_robot_skin_circular(cr)


def cairo_robot_skin_brai(cr):
    with cairo_save(cr):
        h = 0.8
        w = 1.3
        x0 = -.3
        wheel_h = 0.15
        wheel_w = 0.4
        M = h / 2 + .8 * wheel_h

        def body():
            cr.rectangle(x0, -h / 2, w, h)

        cr.move_to(0, M)
        cr.line_to(0, -M)
        cairo_set_color(cr, CairoConstants.BLACK)
        cr.set_line_width(0.05)
        cr.stroke()

        body()
        cairo_set_color(cr, CairoConstants.robot_fill_color)
        cr.fill()
        body()
        cr.set_line_width(CairoConstants.robot_border_width)
        cairo_set_color(cr, CairoConstants.robot_border_color)
        cr.stroke()

        for y in [-M, +M]:
            with cairo_transform(cr, t=[0, y]):
                wheel(cr, w=wheel_w, h=wheel_h)


def cairo_robot_skin_brai_classic(cr):
    with cairo_save(cr):
        h = 0.8
        w = 1.3
        x0 = -.3
        wheel_h = 0.15
        wheel_w = 0.4
        M = h / 2 + .8 * wheel_h

        cairo_robot_skin_brai(cr)

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

