from . import contract, cairo_transform, np, cairo_set_color
import itertools


@contract(x='array[HxWx3](uint8)',
          width='>0')
def cairo_pixels(cr, x, width, height=None, grid_color=[1, .9, .9],
                 border_color=[0, 0, 0], bg_color=[1, 1, 1]):
    x = np.transpose(x, [1, 0, 2]) # I got it wrong below...

    pw = width * 1.0 / x.shape[0] # i = rows and width = x space
    if height is None:
        ph = pw
        height = pw * x.shape[1]
    else:
        ph = height * 1.0 / x.shape[1]

    cr.rectangle(0, 0, width, height)
    cr.set_source_rgb(bg_color[0], bg_color[1], bg_color[2])
    cr.fill()

    bleed = 0.5

    # not sure j, i
    for i, j in itertools.product(range(x.shape[0]),
                                  range(x.shape[1])):
        with cairo_transform(cr, t=[i * pw,
                                    j * ph]):
            col = x[i, j, :] / 255.0

            if np.all(col == bg_color): # XXX: tol?
                # Do not draw, if it is the same color as the background
                continue
            else:
                cr.rectangle(0, 0, pw + bleed, ph + bleed)
                cr.set_source_rgb(col[0], col[1], col[2])
                cr.fill()

    if grid_color is not None:
        cairo_set_color(cr, grid_color)
        for i, j in itertools.product(range(x.shape[0]),
                                      range(x.shape[1])):
            with cairo_transform(cr, t=[i * pw,
                                        j * ph]):
                cr.set_line_width(1)
                cr.rectangle(0, 0, pw, ph)
                cr.stroke()

    cr.rectangle(0, 0, width, height)
    cr.set_line_width(1)
    cairo_set_color(cr, border_color)
    cr.stroke()

    return height


