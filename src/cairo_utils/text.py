from . import cairo_transform


def cairo_text_align(cr, text, halign='left', valign='bottom'):
    extents = cr.text_extents(text)
    width = extents[2]
    height = extents[3]
    if halign == 'center':
        x = -width / 2.0
    elif halign == 'left':
        x = 0
    elif halign == 'right':
        x = -width
    else:
        assert False

    if valign == 'middle':
        y = +height / 2.0
    elif valign == 'bottom':
        y = 0
    elif valign == 'top':
        y = +height
    else:
        assert False

    t = [x, y]
    with cairo_transform(cr, t=t):
        cr.show_text(text)
        cr.stroke()
