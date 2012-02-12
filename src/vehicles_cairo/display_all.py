from . import (cairo_plot_sensor_data, cairo_plot_sources, cairo_save,
    cairo_show_world_geometry, CairoConstants, cairo_set_axis, np,
    cairo_rototranslate, cairo_plot_sensor_skins)
from cairo_utils import cairo_set_color
from contracts import contract
from geometry import angle_from_SE2, translation_from_SE2, SE2_from_SE3, SE3
from vehicles import VehiclesConfig, VehiclesConstants


def vehicles_cairo_display_all(cr, width, height,
                            sim_state,
                            grid=1,
                            zoom=0,
                            bgcolor=[1, 1, 1],
                            zoom_scale_radius=False,
                            extra_draw_world=None,
                            first_person=True,
                            show_world=True,
                            #show_features='relevant',
                            show_sensor_data=True,
                            show_sensor_data_compact=False,
                            show_robot_body=True,
                            show_robot_sensors=True,
                            show_textures=True,
                            plot_sources=False):
    '''
        :param zoom_scale_radius: If true, scales the zoom by the robot radius.
    
    '''
    with cairo_save(cr):
        # Paint white
        if bgcolor is not None:
            with cairo_save(cr):
                cairo_set_color(cr, bgcolor)
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

        world_bounds_pad = 0 # CairoConstants.texture_width
        vehicles_cairo_set_coordinates(cr, width, height,
                                       bounds, robot_pose,
                                       zoom=zoom,
                                       world_bounds_pad=world_bounds_pad,
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

        sensor_types = get_sensor_types(vehicle_state)

        has_cameras = (VehiclesConstants.SENSOR_TYPE_PHOTORECEPTORS
                             in sensor_types)
        has_field_sampler = (VehiclesConstants.SENSOR_TYPE_FIELDSAMPLER
                             in sensor_types)

        if show_world:
            if has_field_sampler:
                cairo_plot_sources(cr, world_state)

            plot_textures = has_cameras and show_textures
            cairo_show_world_geometry(cr, world_state,
                                      plot_textures=plot_textures,
                                      plot_sources=plot_sources,
                                      extra_pad=world_bounds_pad)

        # XXX: tmp
        if extra_draw_world is not None:
            extra_draw_world(cr)

        if show_robot_body:
            joints = get_joints_as_TSE3(vehicle_state)
            extra = vehicle_state.get('extra', {})
            id_skin = extra.get('skin', 'default_skin')
            skin = VehiclesConfig.skins.instance(id_skin) #@UndefinedVariable

            with cairo_rototranslate(cr, robot_pose):
                cr.scale(robot_radius, robot_radius)
                skin.draw(cr, joints=joints,
                          timestamp=sim_state['timestamp'])

        if show_robot_sensors:
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
                                        world_bounds_pad=0,
                                        first_person=False):
    bx = world_bounds[0]
    by = world_bounds[1]

    if zoom == 0:
        m = world_bounds_pad
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

        m = 0 # extra space
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


def get_sensor_types(vehicle_state):
    """ Returns the list of sensor types. """
    return [attached['sensor']['type']
             for attached in vehicle_state['sensors']]

