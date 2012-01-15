from vehicles_cairo import CairoConstants
from vehicles_cairo.utils import (cairo_plot_polyline, cairo_save,
    cairo_plot_circle)
import numpy as np
from vehicles.interfaces.primitives import Source
from vehicles.sensors.field_sampler import get_field_values
import cairo


def cairo_plot_sources(cr, world_state):
    bounds = world_state['bounds']
    primitives = world_state['primitives']

    sources = []
    for s in primitives:
        if s['type'] == 'Source':
            sources.append(Source.from_yaml(s))

    if sources:
        cairo_plot_sources_field(cr,
                       sources=sources,
                       bounds=bounds)


def cairo_show_world_geometry(cr, world_state, plot_sources=False):
    bounds = world_state['bounds']
    primitives = world_state['primitives']

    with cairo_save(cr):
        xb = bounds[0]
        yb = bounds[1]
        cr.rectangle(xb[0], yb[0], xb[1] - xb[0], yb[1] - yb[0])
        cr.clip()

        for p in primitives:
            if p['type'] == 'PolyLine':
                points = np.array(p['points']).T

                with cairo_save(cr):
                    cr.set_line_width(CairoConstants.obstacle_border_width)
                    cr.set_source_rgb(*CairoConstants.obstacle_border_color)
                    cairo_plot_polyline(cr, points[0, :], points[1, :])

            elif p['type'] == 'Circle':
                facecolor = CairoConstants.obstacle_fill_color if p['solid'] else None

                cairo_plot_circle(cr, center=p['center'], radius=p['radius'],
                                  edgecolor=CairoConstants.obstacle_border_color,
                                  facecolor=facecolor,
                                  width=CairoConstants.obstacle_border_width)

            elif p['type'] == 'Source':
                if plot_sources:
                    cairo_plot_circle(cr, center=p['center'], radius=0.05,
                                      edgecolor=[0, 0, 0],
                                      facecolor=[1, 0, 0],
                                      width=0.01)
            else:
                pass # XXX 


def cairo_plot_sources_field(cr, sources, bounds, disc=[100, 100], alpha=0.5,
                       cmap='Greens'):
    if not sources:
        return

    xb = bounds[0]
    yb = bounds[1]

    x = np.linspace(xb[0], xb[1], disc[0])
    y = np.linspace(yb[0], yb[1], disc[1])
    X, Y = np.meshgrid(x, y)

    C = get_field_values(sources, X, Y)
    C = C - C.min()
    C = C / C.max()
    C = 1 - C

    Cscal = ((C) * 255).astype('uint8')

    data = np.empty((disc[0], disc[1], 4), dtype=np.uint8)
    data.fill(255)

#    data[:, :, 1] = Cscal
    data[:, :, 2] = Cscal
#    data[:, :, 3] = Cscal

    image = cairo.ImageSurface.create_for_data(
                        data, cairo.FORMAT_ARGB32, disc[0], disc[1], disc[1] * 4) # XXX

    with cairo_save(cr):
        Z = float(disc[0]) / (xb[1] - xb[0])
        zoom = 1 / Z
        cr.scale(zoom, zoom)
        cr.translate(-disc[0] / 2, -disc[1] / 2)
        cr.set_source_surface(image)
        cr.paint_with_alpha(0.5)

