from . import contract


@contract(col='seq[3]|seq[4]')
def cairo_set_color(cr, col):
    if len(col) == 3:
        cr.set_source_rgb(col[0], col[1], col[2])
    else:
        cr.set_source_rgba(col[0], col[1], col[2], col[3])
