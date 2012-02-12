from . import (CairoConstants as CC, cairo_plot_circle, cairo_rototranslate,
               np, logger, contract)
from cairo_utils import cairo_plot_circle2, cairo_set_color
from geometry import SE2_from_SE3, translation_angle_from_SE2, SE3
from vehicles import VehiclesConfig, VehiclesConstants


def cairo_plot_sensor_skins(cr, vehicle_state, scale=1.0):
    for attached in vehicle_state['sensors']:
        sensor = attached['sensor']
        sensor_pose = SE2_from_SE3(SE3.from_yaml(attached['current_pose']))

        extra = attached.get('extra', {})
        sensor_skin = extra.get('skin', None)

        if sensor_skin is None:
            sensor_skin = sensor['type']

        if not sensor_skin in VehiclesConfig.skins:
            logger.warning('Could not find skin %r' % sensor_skin)
        else:
            skin = VehiclesConfig.specs['skins'].instance(sensor_skin)

            with cairo_rototranslate(cr, sensor_pose):
                cr.scale(scale, scale)
                skin.draw(cr)


def cairo_plot_sensor_data(cr, vehicle_state, scale=1.0, compact=True):
    for attached in vehicle_state['sensors']:
        sensor = attached['sensor']
        observations = attached['current_observations']
        sensor_pose = SE2_from_SE3(SE3.from_yaml(attached['current_pose']))
        sensor_type = sensor['type']

        if sensor_type == VehiclesConstants.SENSOR_TYPE_RANGEFINDER:
            if compact:
                with cairo_rototranslate(cr, sensor_pose):
                    cr.scale(scale, scale)
                    plot_ranges_compact(cr=cr,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings'])
                        )
            else:
                plot_ranges(cr=cr,
                        pose=sensor_pose,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings'])
                        )
        elif sensor_type == VehiclesConstants.SENSOR_TYPE_PHOTORECEPTORS:
            if compact:
                with cairo_rototranslate(cr, sensor_pose):
                    cr.scale(scale, scale)
                    plot_photoreceptors_compact(cr=cr,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        luminance=np.array(observations['luminance']))
            else:
                plot_photoreceptors(cr=cr,
                        pose=sensor_pose,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings']),
                        luminance=np.array(observations['luminance']))
        elif sensor_type == VehiclesConstants.SENSOR_TYPE_FIELDSAMPLER:

            if True:
                with cairo_rototranslate(cr, sensor_pose):
                    plot_fieldsampler_fancy(cr=cr,
                        positions=np.array(sensor['positions']),
                        sensels=np.array(observations['sensels']))
            else:
                plot_fieldsampler(cr=cr,
                        pose=sensor_pose,
                        positions=np.array(sensor['positions']),
                        sensels=np.array(observations['sensels']))
        elif sensor_type == 'RandomSensor':
            # XXX: how can we visualize this?
            pass
        else:
            logger.warning('Unknown sensor type %r.' % sensor['type'])
            pass


@contract(q='SE2', x='array[2]', returns='array[2]')
def SE2_act_R2(q, x):
    return np.dot(q, [x[0], x[1], 1])[:2]


@contract(pose='SE2', positions='array[Nx2]', radius='None|>0',
          sensels='array[N]|(array[JxK],J*K=N)')
def plot_fieldsampler(cr, pose, positions, sensels, radius=None):
    # Scale in [0,1] for better contrast

    sensels = sensels - np.min(sensels)
    if np.max(sensels) > 0:
        sensels = sensels / np.max(sensels)

    # find radius if none given as the minimum distance of the first
    # sensels to the others
    if radius is None:
        radius = find_radius(positions)

    for i, value in enumerate(sensels.flat):
        value = 1 - value
        facecolor = [value, value, value]
        pw = SE2_act_R2(pose, positions[i, :])
        #alpha=0.6
        cairo_plot_circle(cr, center=pw, radius=radius, facecolor=facecolor,
                    edgecolor=None)


def find_radius(positions):
    p0 = positions[0, :]
    pi = positions[1:, :]
    D = np.hypot(pi[:, 0] - p0[0], pi[:, 1] - p0[1])
    radius = D.min() / 2

    radius *= 2  # overlap

    radius = 0.05 # XXX
    return radius

RANDOM_PERM = np.random.permutation(1000)


@contract(positions='array[Nx2]', sensels='array[N]|(array[JxK],J*K=N)',
          radius='None|>0')
def plot_fieldsampler_fancy(cr, positions, sensels, radius=None,
                            alpha=0.2):
    # Scale in [0,1] for better contrast

    sensels = sensels - np.min(sensels)
    if np.max(sensels) > 0:
        sensels = sensels / np.max(sensels)

    # find radius if none given as the minimum distance of the first
    # sensels to the others
    if radius is None:
        radius = find_radius(positions)

    indices = np.array(range(sensels.size))

    # Use completely random permutation
    permutation = np.argsort(RANDOM_PERM[:indices.size])
    # Sort by value
    # permutation = np.argsort(np.array(sensels.flat))

    indices = indices[permutation]

    for i in indices:
        value = sensels.flat[i]
        p = positions[i, :]

        uvalue = value
        facecolor = [uvalue, uvalue, uvalue, value]
        border_color = [0, 0, 0, 0.3 + 0.5 * value]

        radius_i = (alpha + (1 - alpha) * value) * radius

        cairo_plot_circle2(cr, x=p[0], y=p[1], radius=radius_i,
                                fill_color=facecolor,
                                border_color=border_color,
                                border_width=radius / 10.0)


@contract(pose='SE2', directions='array[N]',
          valid='array[N]', readings='array[N]')
def plot_ranges(cr, pose, directions, valid, readings,
                rho_min=CC.plot_ranges_rho_min):
    sensor_t, sensor_theta = translation_angle_from_SE2(pose)
    directions = directions[valid]
    readings = readings[valid]

    for theta_i, rho_i in zip(directions, readings):
        plot_ray(cr, sensor_t, sensor_theta + theta_i,
                 rho1=rho_min,
                 rho2=max(rho_i, rho_min),
                 color=(1, 1, 0)) # XXX


@contract(pose='SE2', directions='array[N]', valid='array[N]',
          readings='array[N]', luminance='array[N]')
def plot_photoreceptors(cr, pose, directions, valid, readings, luminance,
                        rho_min=CC.plot_ranges_rho_min):
    sensor_t, sensor_theta = translation_angle_from_SE2(pose)

    directions = directions[valid]
    readings = readings[valid]
    luminance = luminance[valid]

    for theta_i, rho_i, lum in zip(directions, readings, luminance):
        color = luminance2color(lum)
        plot_ray(cr, sensor_t, sensor_theta + theta_i,
                 rho1=rho_min,
                 rho2=max(rho_i, rho_min),
                 color=color)


def luminance2color(lum):
    # color = (lum, lum, 1) # bluish
    # color = (lum, lum, lum) # normal
    alpha = 0.2
    x = alpha + lum * (1 - 2 * alpha)
    color = (x, x, 1)
    return color


@contract(directions='array[N]', valid='array[N]', luminance='array[N]')
def plot_photoreceptors_compact(cr, directions, valid, luminance,
                                r=CC.photoreceptors_compact_r,
                                r_width=CC.photoreceptors_compact_r_width):
    if len(directions) >= 2:
        delta_theta = np.abs(directions[2] - directions[1])
    else:
        delta_theta = CC.delta_single_ray

    N = directions.size
    for i in range(N):
        if not valid[i]:
            continue

        theta1 = directions[i] - delta_theta / 1.9
        theta2 = directions[i] + delta_theta / 1.9

        r0 = r
        r1 = r + r_width
        cairo_set_color(cr, luminance2color(luminance[i]))
        cairo_fill_slice(cr, theta1, theta2, r0, r1)


@contract(directions='array[N]', valid='array[N]', readings='array[N]')
def plot_ranges_compact(cr, directions, readings, valid,
                        r=CC.laser_compact_r,
                        r_width=CC.laser_compact_r_width):
    if len(directions) >= 2:
        delta_theta = np.abs(directions[2] - directions[1])
    else:
        delta_theta = CC.delta_single_ray

    N = directions.size

    vmax = np.nanmax(readings)
    readings = readings / vmax

    theta1 = directions - delta_theta / 1.9
    theta2 = directions + delta_theta / 1.9
    r0 = r * np.ones(readings.size)
    r1 = r + readings * r_width

    for i in range(N):
        if not valid[i]:
            continue

        cairo_set_color(cr, CC.laser_compact_bg)
        cairo_fill_slice(cr, theta1[i], theta2[i], r, r + r_width)

    for i in range(N):
        if not valid[i]:
            continue

        cairo_set_color(cr, CC.laser_compact_fg)
        cairo_fill_slice(cr, theta1[i], theta2[i], r0[i], r1[i])


def cairo_fill_slice(cr, theta1, theta2, r0, r1):
    Ax = np.cos(theta1) * r0
    Ay = np.sin(theta1) * r0
    Bx = np.cos(theta1) * r1
    By = np.sin(theta1) * r1
    Cx = np.cos(theta2) * r1
    Cy = np.sin(theta2) * r1
    Dx = np.cos(theta2) * r0
    Dy = np.sin(theta2) * r0
    cr.move_to(Ax, Ay)
    cr.line_to(Bx, By)
    cr.line_to(Cx, Cy)
    cr.line_to(Dx, Dy)
    cr.line_to(Ax, Ay)
    cr.fill()


@contract(origin='seq[2](number)', direction='number', rho1='x', rho2='>=x')
def plot_ray(cr, origin, direction, rho1, rho2, color=[0, 0, 0]):
    x = [origin[0] + np.cos(direction) * rho1,
         origin[0] + np.cos(direction) * rho2]
    y = [origin[1] + np.sin(direction) * rho1,
         origin[1] + np.sin(direction) * rho2]
    cr.move_to(x[0], y[0])
    cr.line_to(x[1], y[1])
    cr.set_line_width(0.01) # XXX
    cr.set_source_rgb(*color)
    cr.stroke()


