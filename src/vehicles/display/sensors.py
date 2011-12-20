from . import Z_SENSORS, logger, contract, np, plot_circle
from geometry import SE2_from_SE3, translation_angle_from_SE2
from geometry.yaml import from_yaml


def plot_sensor_data(pylab, vehicle_state, rho_min=0.05):
    for attached in vehicle_state['sensors']:
        sensor = attached['sensor']
        observations = attached['current_observations']
        sensor_pose = SE2_from_SE3(from_yaml(attached['current_pose']))
        if sensor['type'] == 'Rangefinder':
            plot_ranges(pylab=pylab,
                        pose=sensor_pose,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings'])
                        )
        elif sensor['type'] == 'Photoreceptors':
            plot_photoreceptors(pylab=pylab,
                        pose=sensor_pose,
                        directions=np.array(sensor['directions']),
                        valid=np.array(observations['valid']),
                        readings=np.array(observations['readings']),
                        luminance=np.array(observations['luminance']))
        elif sensor['type'] == 'FieldSampler':
            plot_fieldsampler(pylab=pylab,
                        pose=sensor_pose,
                        positions=np.array(sensor['positions']),
                        sensels=np.array(observations['sensels']))
        else:
            logger.warning('Unknown sensor type %r.' % sensor['type'])


@contract(q='SE2', x='array[2]', returns='array[2]')
def SE2_act_R2(q, x):
    return np.dot(q, [x[0], x[1], 1])[:2]


@contract(pose='SE2', positions='array[Nx2]', sensels='array[N]')
def plot_fieldsampler(pylab, pose, positions, sensels, radius=None):
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

    for i, value in enumerate(sensels):
        value = 1 - value
        facecolor = [value, value, value]
        pw = SE2_act_R2(pose, positions[i, :])
        plot_circle(pylab, center=pw, radius=radius, facecolor=facecolor,
                    edgecolor='none', alpha=0.6,
                    zorder=Z_SENSORS, ec='none', lw=0)


@contract(pose='SE2', directions='array[N]',
          valid='array[N]', readings='array[N]')
def plot_ranges(pylab, pose, directions, valid, readings, rho_min=0.05,
                style=dict(zorder=200, markersize=0.5)):
    sensor_t, sensor_theta = translation_angle_from_SE2(pose)
    directions = directions[valid]
    readings = readings[valid]
    for theta_i, rho_i in zip(directions, readings):
        plot_ray(pylab, sensor_t, sensor_theta + theta_i,
                 rho1=rho_min,
                 rho2=rho_i,
                 color='y', **style)


@contract(pose='SE2', directions='array[N]', valid='array[N]',
          readings='array[N]',
          luminance='array[N]')
def plot_photoreceptors(pylab, pose, directions, valid, readings, luminance,
                        rho_min=0.05,
                        style=dict(zorder=200, markersize=0.5)):
    sensor_t, sensor_theta = translation_angle_from_SE2(pose)

    directions = directions[valid]
    readings = readings[valid]
    luminance = luminance[valid]

    for theta_i, rho_i, lum in zip(directions, readings, luminance):
        plot_ray(pylab, sensor_t, sensor_theta + theta_i,
                 rho1=rho_min,
                 rho2=rho_i,
                 color=(lum, lum, lum), **style)


@contract(origin='seq[2](number)', direction='number', rho1='x', rho2='>=x')
def plot_ray(pylab, origin, direction, rho1, rho2, **style):
    x = [origin[0] + np.cos(direction) * rho1,
         origin[0] + np.cos(direction) * rho2]
    y = [origin[1] + np.sin(direction) * rho1,
         origin[1] + np.sin(direction) * rho2]
    pylab.plot(x, y, **style)
