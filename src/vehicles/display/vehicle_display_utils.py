from . import (Z_GRID, Z_POLYLINE, Z_CIRCLE, Z_SOURCE, Z_ROBOT, Z_FIELD,
    plot_sensor_data, np, contract, logger, plot_circle)
from ..interfaces import Source
from ..sensors import get_field_values
from geometry import (SE2_from_SE3, translation_angle_from_SE2,
    translation_from_SE2)
from geometry.yaml import from_yaml


def show_grid(pylab, bx, by, spacing, margin):
    S = spacing    
    M = margin # margin (cells)
    params = dict(markersize=0.5, color=[0.8, 0.8, 1], zorder=Z_GRID)
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

def show_world_geometry(pylab, world_state):
    bounds = world_state['bounds']
    primitives = world_state['primitives']
    for p in primitives:
        if p['type'] == 'PolyLine':
            points = np.array(p['points']).T
            pylab.plot(points[0, :], points[1, :], 'k-', markersize=10,
                       zorder=Z_POLYLINE) 
        elif p['type'] == 'Circle':
            obstacle_color = [0.7, 0.7, 0.8]
            facecolor = obstacle_color if p['solid'] else 'none'
            plot_circle(pylab, center=p['center'], radius=p['radius'],
                        zorder=Z_CIRCLE,
                        edgecolor=obstacle_color, facecolor=facecolor)
            
        elif p['type'] == 'Source':
            if False:
                edgecolor = 'k'
                facecolor = 'r'
                plot_circle(pylab, center=p['center'], radius=0.05, # XXX
                            zorder=Z_SOURCE,
                            edgecolor=edgecolor, facecolor=facecolor)
        else:
            logger.warning('Unknown type %r.' % p['type'])

    sources = []
    for s in primitives:
        if s['type'] == 'Source':
            sources.append(Source.from_yaml(s))

    if sources:
        plot_sources_field(pylab,
                       sources=sources,
                       bounds=bounds)

def plot_sources_field(pylab, sources, bounds, disc=[100, 100], alpha=0.5,
                       cmap='Greens'):
    if not sources:
        logger.warning('No sources given.')
        return 
    xb = bounds[0]
    yb = bounds[1]
    x = np.linspace(xb[0], xb[1], disc[0])
    y = np.linspace(yb[0], yb[1], disc[1])
    X, Y = np.meshgrid(x, y)
#    C = np.empty_like(X)
# Note i,j and j,i --- it's a pylab quirk
#    for i, j in itertools.product(range(disc[0]), range(disc[1])):
#        p = [ X[j, i], Y[j, i]]
#        C[j, i] = get_field_value(sources, p)

    C = get_field_values(sources, X, Y)
    
    pylab.pcolor(X, Y, C, edgecolors='none', alpha=alpha, cmap=cmap,
                 zorder=Z_FIELD)
    
    



@contract(pose='SE2')
def plot_robot(pylab, pose, robot_radius, zorder=Z_ROBOT):       
    t, theta = translation_angle_from_SE2(pose)
    
    plot_circle(pylab, center=t, radius=robot_radius, zorder=zorder,
                edgecolor='k',
                facecolor=[0.5, 0.5, 0.5])
    
    pylab.plot([t[0], t[0] + np.cos(theta) * robot_radius],
               [t[1], t[1] + np.sin(theta) * robot_radius], 'k',
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
        
    bx = bounds[0]
    by = bounds[1]
    if zoom == 0:
        m = 0.5
        m = 0
#        pylab.axis('equal')
        pylab.axis([bx[0] - m, bx[1] + m, by[0] - m, by[1] + m])
        
    else:
        m = zoom
        t = translation_from_SE2(robot_pose)
        
        # don't go over the world side
        t[0] = np.maximum(t[0], bx[0] + m)
        t[0] = np.minimum(t[0], bx[1] - m)
        t[1] = np.maximum(t[1], by[0] + m)
        t[1] = np.minimum(t[1], by[1] - m)
        
        pylab.axis([t[0] - m,
                    t[0] + m,
                    t[1] - m,
                    t[1] + m])
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
