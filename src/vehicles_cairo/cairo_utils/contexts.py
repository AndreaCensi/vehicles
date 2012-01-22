from . import contextmanager


@contextmanager
def cairo_save(cr):
    cr.save()
    yield cr
    cr.restore()
