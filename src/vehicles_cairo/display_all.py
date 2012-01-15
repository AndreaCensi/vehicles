from . import (cairo_plot_sensor_data, cairo_plot_sources, cairo_save,
    cairo_show_world_geometry)
from contracts import contract
from geometry import SE2, translation_from_SE2, SE2_from_SE3
from geometry.yaml import from_yaml
from vehicles_cairo import CairoConstants
import geometry
import numpy as np


def vehicles_cairo_display_png(filename, width, height, sim_state,
                               **plotting_params):
    import cairo #@UnresolvedImport
    surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable

    vehicles_cairo_display_all(cr, width, height, sim_state, **plotting_params)

    surf.write_to_png(filename) # Output to PNG


def vehicles_cairo_display_pdf(filename, width, height, sim_state,
                                **plotting_params):
    import cairo #@UnresolvedImport
    surf = cairo.PDFSurface(filename, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable
    vehicles_cairo_display_all(cr, width, height, sim_state, **plotting_params)
    surf.show_page()
    surf.finish()


def vehicles_cairo_display_svg(filename, width, height, sim_state,
                                **plotting_params):
    import cairo #@UnresolvedImport
    surf = cairo.SVGSurface(filename, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable
    vehicles_cairo_display_all(cr, width, height, sim_state, **plotting_params)
    surf.show_page()
    surf.finish()


def vehicles_cairo_display_all(cr, width, height,
                            sim_state, grid=1, zoom=0, show_sensor_data=True):
    import cairo
    # Paint white
    with cairo_save(cr):
        cr.set_source_rgb(1.0, 1.0, 1)
        cr.set_operator(cairo.OPERATOR_SOURCE) #@UndefinedVariable
        cr.paint()

    if False:
        cr.set_source_rgb(1, 0, 0)
        cr.rectangle(5, 5, width - 5, height - 5)
        cr.stroke()

    vehicle_state = sim_state['vehicle']
    robot_pose = SE2_from_SE3(from_yaml(vehicle_state['pose']))

    robot_radius = vehicle_state['radius']
    world_state = sim_state['world']
    bounds = world_state['bounds']
    bx = bounds[0]
    by = bounds[1]

    vehicles_cairo_set_coordinates(cr, width, height, bounds, robot_pose, zoom)

    if False:
        cr.set_source_rgb(0, 1, 0)
        cr.set_line_width(0.05)
        cr.rectangle(bx[0], by[0], bx[1] - bx[0], by[1] - by[0])
        cr.stroke()

    if grid > 0:
        show_grid(cr, bx, by, spacing=grid, margin=1)

    cairo_plot_sources(cr, world_state)

    cairo_show_world_geometry(cr, world_state)
#
    display_robot = True
    if display_robot:
        with cairo_save(cr):
            rototranslate(cr, robot_pose)
            cr.scale(robot_radius, robot_radius)

            cr.set_line_width(0.01)
            cr.arc(0, 0, 1, 0, 2 * np.pi)
            cr.set_source_rgb(0.5, 0.5, 0.5)
            cr.fill()
            cr.arc(0, 0, 1, 0, 2 * np.pi)
            cr.set_source_rgb(0, 0, 0)
            cr.stroke()
            cr.move_to(0, 0)
            cr.line_to(1, 0)
            cr.stroke()

    if show_sensor_data:
        cairo_plot_sensor_data(cr, vehicle_state, rho_min=robot_radius)


@contract(a='seq[4](number)')
def cairo_set_axis(cr, width, height, a):
    xmin, xmax, ymin, ymax = a
    cr.translate(width / 2, height / 2)
    Z = max(float(xmax - xmin) / width,
            float(ymax - ymin) / height)
    assert Z > 0
    cr.scale(1 / Z, -1 / Z)
    xmid = (xmin + xmax) / 2.0
    ymid = (ymin + ymax) / 2.0
    cr.translate(-xmid, -ymid)


@contract(zoom='>=0')
def vehicles_cairo_set_coordinates(cr, width, height, world_bounds,
                                        robot_pose, zoom):
    bx = world_bounds[0]
    by = world_bounds[1]

    m = 0 # extra space

    if zoom == 0:
        extents = [bx[0] - m,
                   bx[1] + m,
                   by[0] - m,
                   by[1] + m]
    else:
        t = translation_from_SE2(robot_pose)
        # don't go over the world side
        t[0] = np.maximum(t[0], bx[0] + zoom)
        t[0] = np.minimum(t[0], bx[1] - zoom)
        t[1] = np.maximum(t[1], by[0] + zoom)
        t[1] = np.minimum(t[1], by[1] - zoom)

        extents = [t[0] - zoom - m,
                   t[0] + zoom + m,
                   t[1] - zoom - m,
                   t[1] + zoom + m]

    cairo_set_axis(cr, width, height, extents)


def rototranslate(cr, pose):
    #print('rototranslating at %s' % SE2.friendly(pose))
    translation, rotation = geometry.translation_angle_from_SE2(pose)
    cr.translate(translation[0], translation[1])
    cr.rotate(rotation)


def show_grid(cr, bx, by, spacing, margin):
    cr.save()

    cr.set_source_rgb(*CairoConstants.grid_color)
    cr.set_line_width(CairoConstants.grid_width)

    def segment(cr, x0, x1, y0, y1):
        cr.move_to(x0, y0)
        cr.line_to(x1, y1)
        cr.stroke()

    S = spacing
    M = margin  # margin (cells)
    M = 0
    xmin = (np.floor(bx[0] / S) - M) * S
    xmax = (np.ceil(bx[1] / S) + M) * S
    ymin = (np.floor(by[0] / S) - M) * S
    ymax = (np.ceil(by[1] / S) + M) * S

    for x in np.linspace(xmin, xmax, np.round((xmax - xmin) / S) + 1):
        segment(cr, x, x, ymin, ymax)

    for y in np.linspace(ymin, ymax, np.round((ymax - ymin) / S) + 1):
        segment(cr, xmin, xmax, y, y)

    cr.restore()
