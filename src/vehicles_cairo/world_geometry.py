from . import (cairo_plot_polyline, cairo_save, cairo_plot_circle, cairo,
    CairoConstants, cairo_set_color, np)
from conf_tools import instantiate_spec
from vehicles.interfaces import Source
from vehicles.sensors.field_sampler import get_field_values
from vehicles_cairo.cairo_utils import cairo_transform


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


def cairo_plot_texture_circle(cr, circle_yaml, resolution, width_inside, width_outside):
    texture = instantiate_spec(circle_yaml['texture'])
    radius = circle_yaml['radius']
    center = circle_yaml['center']
    diameter = np.pi * 2 * radius
    npoints = int(np.ceil(diameter / resolution))
    theta = np.linspace(0, np.pi * 2, npoints)
    t = theta * radius
    values = texture(t)

    resolution_angle = np.pi * 2 / npoints

#    print('REsolution: %s  w: %s radius: %s points: %s angle: %sdeg' % (resolution, width, radius, npoints,
#                                                          np.rad2deg(resolution_angle)))


    theta1 = theta - resolution_angle / 1.9 # 10% overlap
    theta2 = theta + resolution_angle / 1.9

    r0 = radius - width_inside
    r1 = radius + width_outside

    with cairo_transform(cr, t=center):
        Ax = np.cos(theta1) * r0
        Ay = np.sin(theta1) * r0
        Bx = np.cos(theta1) * r1
        By = np.sin(theta1) * r1
        Cx = np.cos(theta2) * r1
        Cy = np.sin(theta2) * r1
        Dx = np.cos(theta2) * r0
        Dy = np.sin(theta2) * r0

        for i in range(len(t)):
            cr.set_source_rgb(values[i], values[i], values[i])

            cr.move_to(Ax[i], Ay[i])
            cr.line_to(Bx[i], By[i])
            cr.line_to(Cx[i], Cy[i])
            cr.line_to(Dx[i], Dy[i])
            cr.line_to(Ax[i], Ay[i])
            cr.fill()

#            color = [values[i], values[i], values[i]]

#            cairo_plot_circle2(cr, x[i], y[i], resolution, fill_color=color)


CC = CairoConstants

def cairo_show_world_geometry(cr, world_state, plot_sources=False):
    bounds = world_state['bounds']
    primitives = world_state['primitives']


    with cairo_save(cr):
        xb = bounds[0]
        yb = bounds[1]
        cr.rectangle(xb[0], yb[0], xb[1] - xb[0], yb[1] - yb[0])
        cr.clip()

        for i in range(2):
            for p in primitives:
                if p['type'] == 'PolyLine':
                    points = np.array(p['points']).T

                    with cairo_save(cr):
                        cr.set_line_width(CC.obstacle_border_width)
                        cairo_set_color(cr,
                                        CC.obstacle_border_color)
                        cairo_plot_polyline(cr, points[0, :], points[1, :])

                elif p['type'] == 'Circle':

                    facecolor = (CairoConstants.obstacle_fill_color
                                 if p['solid'] else None)

                    if i == 0:
                        cairo_plot_circle(cr, center=p['center'],
                                      radius=p['radius'],
                                      edgecolor=CC.obstacle_border_color,
                                    facecolor=facecolor,
                                    width=CC.obstacle_border_width)

                        if not p['solid']:
                            cairo_plot_circle(cr, center=p['center'],
                                      radius=p['radius'] + CC.texture_width,
                                      edgecolor=[0, 0, 0],
                                      width=CC.obstacle_border_width)

                        if p['solid']:
                            cairo_plot_texture_circle(cr, p,
                                      resolution=CC.texture_resolution,
                                      width_inside=CC.texture_width,
                                      width_outside=0)
                        else:
                            cairo_plot_texture_circle(cr, p,
                                      resolution=CC.texture_resolution,
                                      width_inside=0,
                                      width_outside=CC.texture_width)

                    else:
                        cairo_plot_circle(cr, center=p['center'],
                                      radius=p['radius'] - CC.texture_width,
                                      edgecolor=None,
                                      facecolor=facecolor,
                                      width=CC.obstacle_border_width)


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

    image = cairo.ImageSurface.create_for_data(#@UndefinedVariable
                        data, cairo.FORMAT_ARGB32, #@UndefinedVariable
                         disc[0], disc[1], disc[1] * 4)

    with cairo_save(cr):
        Z = float(disc[0]) / (xb[1] - xb[0])
        zoom = 1 / Z
        cr.scale(zoom, zoom)
        cr.translate(-disc[0] / 2, -disc[1] / 2)
        cr.set_source_surface(image)
        cr.paint_with_alpha(0.5)

