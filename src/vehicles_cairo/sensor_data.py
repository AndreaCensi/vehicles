import numpy as np
from geometry.yaml import from_yaml
from geometry.poses import SE2_from_SE3, translation_angle_from_SE2
from contracts import contract
from vehicles_cairo.utils import cairo_plot_circle
from geometry.manifolds import SE2


def cairo_plot_sensor_data(cr, vehicle_state, rho_min=0.05):
    for attached in vehicle_state['sensors']:
        sensor = attached['sensor']
        observations = attached['current_observations']
        sensor_pose = SE2_from_SE3(from_yaml(attached['current_pose']))
        if sensor['type'] == 'Rangefinder':
            plot_ranges(cr=cr,
                        pose=sensor_pose,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings']),
                        rho_min=rho_min
                        )
        elif sensor['type'] == 'Photoreceptors':
            print 'sensor_pose: %s' % SE2.friendly(sensor_pose)
            plot_photoreceptors(cr=cr,
                        pose=sensor_pose,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings']),
                        luminance=np.array(observations['luminance']),
                        rho_min=rho_min)
        elif sensor['type'] == 'FieldSampler':
            plot_fieldsampler(cr=cr,
                        pose=sensor_pose,
                        positions=np.array(sensor['positions']),
                        sensels=np.array(observations['sensels']))
        else:
            pass # XXX
            #logger.warning('Unknown sensor type %r.' % sensor['type'])


@contract(q='SE2', x='array[2]', returns='array[2]')
def SE2_act_R2(q, x):
    return np.dot(q, [x[0], x[1], 1])[:2]


@contract(pose='SE2', positions='array[Nx2]',
          sensels='array[N]|(array[JxK],J*K=N)')
def plot_fieldsampler(cr, pose, positions, sensels, radius=None):
    # Scale in [0,1] for better contrast

    sensels = sensels - np.min(sensels)
    if np.max(sensels) > 0:
        sensels = sensels / np.max(sensels)

    # find radius if none given as the minimum distance of the first
    # sensels to the others
    if radius is None:
        p0 = positions[0, :]
        pi = positions[1:, :]
        D = np.hypot(pi[:, 0] - p0[0], pi[:, 1] - p0[1])
        radius = D.min() / 2

        radius *= 2  # overlap

        radius = 0.05

    for i, value in enumerate(sensels.flat):
        value = 1 - value
        facecolor = [value, value, value]
        pw = SE2_act_R2(pose, positions[i, :])
        #alpha=0.6
        cairo_plot_circle(cr, center=pw, radius=radius, facecolor=facecolor,
                    edgecolor=None)


@contract(pose='SE2', directions='array[N]',
          valid='array[N]', readings='array[N]')
def plot_ranges(cr, pose, directions, valid, readings, rho_min=0.05):
    sensor_t, sensor_theta = translation_angle_from_SE2(pose)
    directions = directions[valid]
    readings = readings[valid]

    print('directions', np.rad2deg(directions))
    print('readings', readings)

    t = np.rad2deg(directions)
#    print t[1:] - t[:-1]
#    print t[0] - t[-1] + 360
    for theta_i, rho_i in zip(directions, readings):
        plot_ray(cr, sensor_t, sensor_theta + theta_i,
                 rho1=rho_min,
                 rho2=rho_i,
                 color=(1, 1, 0))


@contract(pose='SE2', directions='array[N]', valid='array[N]',
          readings='array[N]',
          luminance='array[N]')
def plot_photoreceptors(cr, pose, directions, valid, readings, luminance,
                        rho_min=0.05):
    sensor_t, sensor_theta = translation_angle_from_SE2(pose)

    directions = directions[valid]
    readings = readings[valid]
    luminance = luminance[valid]

    for theta_i, rho_i, lum in zip(directions, readings, luminance):
        plot_ray(cr, sensor_t, sensor_theta + theta_i,
                 rho1=rho_min,
                 rho2=rho_i,
                 color=(lum, lum, lum))


@contract(origin='seq[2](number)', direction='number', rho1='x', rho2='>=x')
def plot_ray(cr, origin, direction, rho1, rho2, color=[0, 0, 0]):
    x = [origin[0] + np.cos(direction) * rho1,
         origin[0] + np.cos(direction) * rho2]
    y = [origin[1] + np.sin(direction) * rho1,
         origin[1] + np.sin(direction) * rho2]
    cr.move_to(x[0], y[0])
    cr.line_to(x[1], y[1])
    cr.set_line_width(0.01)
    cr.set_source_rgb(*color)
    cr.stroke()

