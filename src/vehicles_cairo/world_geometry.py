from . import cairo, CairoConstants, np, contract
from cairo_utils import (cairo_transform, cairo_plot_circle,
                         cairo_plot_polyline, cairo_save, cairo_set_color)
from conf_tools import instantiate_spec
from vehicles import Source, VehiclesConstants
from vehicles.library.sensors import get_field_values


def cairo_plot_sources(cr, world_state):
    bounds = world_state['bounds']
    primitives = world_state['primitives']

    sources = []
    for s in primitives:
        if s['type'] == VehiclesConstants.PRIMITIVE_SOURCE:
            sources.append(Source.from_yaml(s))

    if sources:
        cairo_plot_sources_field(cr,
                       sources=sources,
                       bounds=bounds)


def cairo_plot_texture_circle(cr, radius, center, texture, resolution,
                              width_inside, width_outside):
    diameter = np.pi * 2 * radius
    npoints = int(np.ceil(diameter / resolution))
    theta = np.linspace(0, np.pi * 2, npoints)
    t = theta * radius
    values = texture(t)

    resolution_angle = np.pi * 2 / npoints

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


CC = CairoConstants


@contract(center='seq[2](float)', radius='>0', solid='bool', numpass='int')
def cairo_plot_circle_primitive(cr, center, radius, solid, texture, numpass):
    facecolor = (CairoConstants.obstacle_fill_color if solid else None)
    if numpass == 0:
        cairo_plot_circle(cr, center=center,
                      radius=radius,
                      edgecolor=CC.obstacle_border_color,
                    facecolor=facecolor,
                    width=CC.obstacle_border_width)

        if not solid:
            cairo_plot_circle(cr, center=center,
                      radius=radius + CC.texture_width,
                      edgecolor=[0, 0, 0],
                      width=CC.obstacle_border_width)

        if texture is not None:
            if solid:
                width_outside = 0
                width_inside = CC.texture_width
            else:
                width_inside = 0
                width_outside = CC.texture_width

            cairo_plot_texture_circle(cr=cr,
                                      radius=radius, center=center,
                                      texture=texture,
                                      resolution=CC.texture_resolution,
                                      width_inside=width_inside,
                                      width_outside=width_outside)

    else:
        r1 = radius - CC.texture_width
        if r1 > 0:
            cairo_plot_circle(cr, center=center,
                      radius=r1,
                      edgecolor=None,
                      facecolor=facecolor,
                      width=CC.obstacle_border_width)


@contract(points='array[2xN]')
def cairo_plot_polyline_primitive(cr, points, texture=None):

    with cairo_save(cr):
        cr.set_line_width(CC.obstacle_border_width)
        cairo_set_color(cr,
                        CC.obstacle_border_color)
        cairo_plot_polyline(cr, points[0, :], points[1, :])

    if texture is not None:
        cairo_plot_textured_polyline(cr, points=points, texture=texture,
                                  resolution=CC.texture_resolution,
                                  width_inside=0,
                                  width_outside=CC.texture_resolution)


@contract(points='array[2xN]')
def cairo_plot_textured_polyline(cr, points, texture,
                                  resolution,
                                  width_inside, width_outside):
    n = points.shape[1]
    delta_t = 0.0
    for i in range(n - 1):
        p1 = points[:, i]
        p2 = points[:, i + 1]
        d = p2 - p1
        segment_len = np.linalg.norm(d)
        angle = np.arctan2(d[1], d[0])

        with cairo_transform(cr, t=p1, r=angle):
            cairo_plot_textured_segment(cr=cr,
                                        length=segment_len, texture=texture,
                                        resolution=resolution, offset=delta_t,
                                        width_inside=width_inside,
                                        width_outside=width_outside)

        delta_t += segment_len


@contract(length='float,>0', resolution='float,>0', offset='float',
          width_inside='>=0', width_outside='>=0')
def cairo_plot_textured_segment(cr, texture, length, resolution, offset,
                          width_inside, width_outside):
    npoints = int(np.ceil(length / resolution))
    cell_size = length / npoints
    t = np.linspace(0, length, npoints) - offset
    values = texture(t)
    for i in range(npoints - 1):
        x0 = i * cell_size
        x1 = (i + 1) * cell_size
        if i > 0:
            x0 -= cell_size * 0.05 # overlap
        y0 = -width_inside
        y1 = +width_outside

        cr.set_source_rgb(values[i], values[i], values[i])
        cr.rectangle(x0, y0, x1 - x0, y1 - y0)
        cr.fill()


def cairo_show_world_geometry(cr, world_state,
                              plot_sources=False,
                              plot_textures=False,
                              extra_pad=0):
    bounds = world_state['bounds']
    primitives = world_state['primitives']

    with cairo_save(cr):
        xb = bounds[0]
        yb = bounds[1]
        xb = [xb[0] - extra_pad, xb[1] + extra_pad]
        yb = [yb[0] - extra_pad, yb[1] + extra_pad]

        cr.rectangle(xb[0], yb[0], xb[1] - xb[0], yb[1] - yb[0])
        cr.clip()

        # Draw it twice for special effects
        for i in range(2):
            for p in primitives:
                ptype = p['type']
                if ptype == VehiclesConstants.PRIMITIVE_POLYLINE:
                    points = np.array(p['points']).T
                    texture = instantiate_spec(p['texture'])
                    if not plot_textures:
                        texture = None
                    cairo_plot_polyline_primitive(cr, points=points,
                                                  texture=texture)

                elif ptype == VehiclesConstants.PRIMITIVE_CIRCLE:
                    texture = instantiate_spec(p['texture'])
                    if not plot_textures:
                        texture = None
                    cairo_plot_circle_primitive(cr, center=p['center'],
                                                texture=texture,
                                                radius=p['radius'],
                                                solid=p['solid'],
                                                numpass=i)

                elif ptype == VehiclesConstants.PRIMITIVE_SOURCE:
                    if plot_sources:
                        cairo_plot_circle(cr, center=p['center'],
                                          radius=0.05,
                                          edgecolor=[0, 0, 0],
                                          facecolor=[1, 0, 0],
                                          width=0.01)
                    # TODO: parametrize
                else:
                    pass # XXX 


def cairo_plot_sources_field(cr, sources, bounds,
                             disc=[100, 100], alpha=0.5):
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

    data[:, :, 2] = Cscal

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

