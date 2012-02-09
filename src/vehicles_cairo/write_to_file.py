from . import cairo, vehicles_cairo_display_all
import os
import subprocess


def vehicles_cairo_display_png(filename, width, height, sim_state,
                               trim=True,
                               **plotting_params):
    surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, #@UndefinedVariable 
                              width, height)
    cr = cairo.Context(surf) #@UndefinedVariable

    vehicles_cairo_display_all(cr, width,
                               height, sim_state, **plotting_params)

    if trim:
        tmp = filename + '.tmp.png'
        surf.write_to_png(tmp)
        try:
            cmd = ['convert', tmp, '-trim', filename]
            subprocess.check_call(cmd)
        except Exception as e:
            print('Could not trim: %s' % e) # XXX
            surf.write_to_png(filename) # Output to PNG
        if os.path.exists(tmp):
            os.unlink(tmp)
    else:
        surf.write_to_png(filename) # Output to PNG


def vehicles_cairo_display_pdf(filename, width, height, sim_state,
                                **plotting_params):
    surf = cairo.PDFSurface(filename, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable
    vehicles_cairo_display_all(cr, width, height, sim_state,
                               **plotting_params)
    surf.show_page()
    surf.finish()


def vehicles_cairo_display_svg(filename, width, height, sim_state,
                                **plotting_params):
    surf = cairo.SVGSurface(filename, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable
    vehicles_cairo_display_all(cr, width, height, sim_state,
                               **plotting_params)
    surf.show_page()
    surf.finish()
