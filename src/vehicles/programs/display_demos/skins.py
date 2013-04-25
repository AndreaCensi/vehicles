from ... import logger
from optparse import OptionParser
from reprep import MIME_PNG, Report
from vehicles import VehiclesConfig
from vehicles_cairo import vehicles_has_cairo
import os


usage = """

    %cmd  [-o where] 
"""


def main():
    if not vehicles_has_cairo:
        logger.error('This program cannot be run if Cairo is not installed.')
        return
    from vehicles_cairo import (cairo_plot_rectangle)
    from vehicles_cairo import  cairo_plot_circle, cairo_set_axis, show_grid

    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()

    parser.add_option("--outdir", "-o", default='.',
                    help="output directory [%default]")

#    parser.add_option("--outdir", "-o", default='.',
#                    help="additional config directory [%default]")

    (options, args) = parser.parse_args()
    if args:
        raise Exception()

    width = 400
    height = 400

    r = Report('skins_demo')
    f = r.figure(cols=3)

    import cairo

    VehiclesConfig.make_sure_loaded()

    skins = VehiclesConfig.specs['skins'].keys()
    logger.info('Skins: %s' % skins)
    for id_skin in skins:
        skin = VehiclesConfig.specs['skins'].instance(id_skin)
        with f.data_file(id_skin, MIME_PNG) as filename:
            surf = cairo.ImageSurface(cairo.FORMAT_ARGB32,  # @UndefinedVariable 
                                      width, height)
            cr = cairo.Context(surf)  # @UndefinedVariable
            cairo_plot_rectangle(cr, 0, 0, width, height, fill_color=[1, 1, 1])

            cairo_set_axis(cr, width, height, [-2, +2, -2, +2])

            cairo_plot_circle(cr, center=[0, 0], radius=1, facecolor=None,
                    edgecolor=(1, 0, 0), width=0.1)

            show_grid(cr, bx=[-3, +3], by=[-3, +3], spacing=1.0)

            if skin.njoints_required() > 1:
                logger.warning('Skipping skin %r' % id_skin)
                continue

            try:
                skin.draw(cr, joints=[])
            except Exception:
                logger.error('Error for skin %r' % id_skin)
                raise

            surf.write_to_png(filename)  # Output to PNG

    filename = os.path.join(options.outdir, 'index.html')
    logger.info('Writing to %r.' % filename)
    r.to_html(filename)


if __name__ == '__main__':
    main()
