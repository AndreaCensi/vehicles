from . import cairo_save, contextmanager


@contextmanager
def cairo_transform(cr, t=[0, 0], r=0, s=1):
    with cairo_save(cr):
        cr.translate(t[0], t[1])
        cr.rotate(r)
        cr.scale(s, s)
        yield cr

