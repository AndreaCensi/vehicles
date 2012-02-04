from . import (cairo_plot_sensor_data, cairo_plot_sources, cairo_save,
    cairo_show_world_geometry, cairo, CairoConstants, cairo_set_axis, np,
    cairo_rototranslate, cairo_plot_sensor_skins)
from contracts import contract
from geometry import angle_from_SE2, translation_from_SE2, SE2_from_SE3, SE3
from vehicles import VehiclesConfig


def vehicles_cairo_display_png(filename, width, height, sim_state,
                               **plotting_params):
    surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, #@UndefinedVariable 
                              width, height)
    cr = cairo.Context(surf) #@UndefinedVariable

    vehicles_cairo_display_all(cr, width, height, sim_state, **plotting_params)

    surf.write_to_png(filename) # Output to PNG


def vehicles_cairo_display_pdf(filename, width, height, sim_state,
                                **plotting_params):
    surf = cairo.PDFSurface(filename, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable
    vehicles_cairo_display_all(cr, width, height, sim_state, **plotting_params)
    surf.show_page()
    surf.finish()


def vehicles_cairo_display_svg(filename, width, height, sim_state,
                                **plotting_params):
    surf = cairo.SVGSurface(filename, width, height) #@UndefinedVariable
    cr = cairo.Context(surf) #@UndefinedVariable
    vehicles_cairo_display_all(cr, width, height, sim_state, **plotting_params)
    surf.show_page()
    surf.finish()


def vehicles_cairo_display_all(cr, width, height,
                            sim_state,
                            grid=1,
                            zoom=0,
                            zoom_scale_radius=False,
                            extra_draw_world=None,
                            first_person=True,
                            show_world=True,
                            #show_features='relevant',
                            show_sensor_data=True,
                            show_sensor_data_compact=False):
    '''
        :param zoom_scale_radius: If true, scales the zoom by the robot radius.
    
    '''
    with cairo_save(cr):
        # Paint white
        with cairo_save(cr):
            cr.set_source_rgb(1, 1, 1)
            cr.rectangle(0, 0, width, height)
            cr.fill()

        vehicle_state = sim_state['vehicle']
        robot_pose = SE2_from_SE3(SE3.from_yaml(vehicle_state['pose']))

        robot_radius = vehicle_state['radius']
        world_state = sim_state['world']
        bounds = world_state['bounds']
        bx = bounds[0]
        by = bounds[1]

        if zoom_scale_radius and zoom != 0:
            zoom = zoom * robot_radius

        vehicles_cairo_set_coordinates(cr, width, height,
                                       bounds, robot_pose,
                                       zoom=zoom,
                                       first_person=first_person)

        if False:
            cr.set_source_rgb(0, 1, 0)
            cr.set_line_width(0.05)
            cr.rectangle(bx[0], by[0], bx[1] - bx[0], by[1] - by[0])
            cr.stroke()

        if grid > 0:
            show_grid(cr, bx, by, spacing=grid, margin=1)

        if extra_draw_world is not None:
            extra_draw_world(cr)

        if show_world:
            cairo_plot_sources(cr, world_state)
            cairo_show_world_geometry(cr, world_state)

        # XXX: tmp
        if extra_draw_world is not None:
            extra_draw_world(cr)
#
        display_robot = True
        if display_robot:
            joints = get_joints_as_TSE3(vehicle_state)
            extra = vehicle_state.get('extra', {})
            id_skin = extra.get('skin', 'default_skin')
            skin = VehiclesConfig.skins.instance(id_skin) #@UndefinedVariable

            with cairo_rototranslate(cr, robot_pose):
                cr.scale(robot_radius, robot_radius)
                skin.draw(cr, joints=joints,
                          timestamp=sim_state['timestamp'])

        # don't like it cause it uses global "current_pose"
        cairo_plot_sensor_skins(cr, vehicle_state,
                               scale=robot_radius)

        if show_sensor_data:
            cairo_plot_sensor_data(cr, vehicle_state,
                                   scale=robot_radius,
                                   compact=show_sensor_data_compact)


def get_joints_as_TSE3(vehicle_state):
    joints = []
    for js in vehicle_state['joints']:
        joints.append(SE3.from_yaml(js))
    return joints


@contract(zoom='>=0')
def vehicles_cairo_set_coordinates(cr, width, height, world_bounds,
                                        robot_pose, zoom,
                                        first_person=False):
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
        if not first_person:
            t[0] = np.maximum(t[0], bx[0] + zoom)
            t[0] = np.minimum(t[0], bx[1] - zoom)
            t[1] = np.maximum(t[1], by[0] + zoom)
            t[1] = np.minimum(t[1], by[1] - zoom)

        extents = [t[0] - zoom - m,
                   t[0] + zoom + m,
                   t[1] - zoom - m,
                   t[1] + zoom + m]

    cairo_set_axis(cr, width, height, extents)

    if first_person and zoom != 0:
        angle = angle_from_SE2(robot_pose)
        t = translation_from_SE2(robot_pose)
        cr.translate(t[0], t[1])
        cr.rotate(-angle)
        cr.rotate(+np.pi / 2) # make the robot point "up"
        cr.translate(-t[0], -t[1])


def show_grid(cr, bx, by, spacing=1, margin=0):
    cr.save()

    cr.set_source_rgb(*CairoConstants.grid_color)
    cr.set_line_width(CairoConstants.grid_width)

    def segment(cr, x0, x1, y0, y1):
        cr.move_to(x0, y0)
        cr.line_to(x1, y1)
        cr.stroke()

    S = spacing
    #M = margin  # margin (cells)
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
