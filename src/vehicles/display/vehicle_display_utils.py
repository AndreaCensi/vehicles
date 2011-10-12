from contracts import contract
from geometry import (SE2_from_SE3, translation_angle_from_SE2,
    translation_from_SE2)
from geometry.yaml import from_yaml
import numpy as np


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
        else:
            print('Unknown sensor type %r' % sensor['type']) # XXX

def show_grid(pylab, bx, by, spacing, margin):
    S = spacing    
    M = margin # margin (cells)
    params = dict(markersize=0.5, color=[0.8, 0.8, 1], zorder= -1000)
    xmin = (np.floor(bx[0] / S) - M) * S
    xmax = (np.ceil(bx[1] / S) + M) * S
    ymin = (np.floor(by[0] / S) - M) * S
    ymax = (np.ceil(by[1] / S) + M) * S
    px = []
    py = []
    for x in np.linspace(xmin, xmax, np.round((xmax - xmin) / S) + 1):
        px.extend([x, x, None])
        py.extend([ymin, ymax, None])
        
    for y in np.linspace(ymin, ymax, np.round((ymax - ymin) / S) + 1):
        px.extend([xmin, xmax, None])
        py.extend([y, y, None])

    pylab.plot(px, py, **params)

@contract(pose='SE2', directions='array[N]', valid='array[N]', readings='array[N]')
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

    
@contract(pose='SE2', directions='array[N]', valid='array[N]', readings='array[N]',
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
    x = [origin[0] + np.cos(direction) * rho1 ,
         origin[0] + np.cos(direction) * rho2 ]
    y = [origin[1] + np.sin(direction) * rho1 ,
         origin[1] + np.sin(direction) * rho2 ]
    pylab.plot(x, y, **style)
    

def show_world_geometry(pylab, world_state, zorder=1000):
    for p in world_state['primitives']:
        if p['type'] == 'PolyLine':
            points = np.array(p['points']).T
            pylab.plot(points[0, :], points[1, :], 'k-', markersize=10) 
        elif p['type'] == 'Circle':
            facecolor = [0.4, 0.4, 0.4] if p['solid'] else 'none'
            cir = pylab.Circle(p['center'], radius=p['radius'], zorder=zorder,
                               edgecolor=[0.4, 0.4, 0.4], facecolor=facecolor)
            pylab.gca().add_patch(cir)

        else:
            print('Unknown type %r.' % p['type'])


@contract(pose='SE2')
def plot_robot(pylab, pose, robot_radius, zorder=1002):       
    t, theta = translation_angle_from_SE2(pose)
    cir = pylab.Circle((t[0], t[1]), radius=robot_radius, zorder=1001,
                       edgecolor='k', facecolor=[0.5, 0.5, 0.5])
    pylab.gca().add_patch(cir)
    pylab.plot([t[0], t[0] + np.cos(theta) * robot_radius * 3],
               [t[1], t[1] + np.sin(theta) * robot_radius * 3], 'k',
               zorder=zorder)



def display_all(pylab, state, grid=0, zoom=0, show_sensor_data=True):
    vehicle_state = state['vehicle']
    world_state = state['world']
    
    bounds = world_state['bounds']
    boundsx = bounds[0]
    boundsy = bounds[1]
    
    robot_pose = SE2_from_SE3(from_yaml(vehicle_state['pose']))
    robot_radius = vehicle_state['radius']
    
    if grid > 0:
        show_grid(pylab, boundsx, boundsy, spacing=grid, margin=1)
            
    show_world_geometry(pylab, world_state)
    
    if show_sensor_data:
        plot_sensor_data(pylab, vehicle_state)
    
    plot_robot(pylab, robot_pose, robot_radius)
        
    if zoom == 0:
        bx = bounds[0]
        by = bounds[1]
        m = 0.5
        m = 0
#        pylab.axis('equal')
        pylab.axis([bx[0] - m, bx[1] + m, by[0] - m, by[1] + m])
        
    else:
        m = zoom
        t = translation_from_SE2(robot_pose)
        pylab.axis([t[0] - m, t[0] + m, t[1] - m, t[1] + m])
        #  pylab.axis('equal')
        
    if False:
        pylab.plot([0, 2], [0, 0], 'r-')
        pylab.text(2, 0, 'x')
        pylab.plot([0, 0], [0, 2], 'g-')
        pylab.text(0, 2, 'y')
    # turn off ticks labels
    axes = pylab.gca()
    pylab.setp(axes.get_xticklabels(), visible=False)
    pylab.setp(axes.get_yticklabels(), visible=False)
