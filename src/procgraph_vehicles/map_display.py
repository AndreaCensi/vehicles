from geometry import SE2_from_SE3, translation_angle_from_SE2, SE2
from geometry.yaml import from_yaml
from procgraph import Block
from procgraph_mpl import pylab2rgb, pylab
import numpy as np

class WorldDisplay(Block):
    ''' Produces a top-down plot of a circular arena.
    '''
    
    Block.alias('world_display')
    
    Block.config('width', 'Image width in pixels.', default=320)
     
    Block.input('state', 'Simulation state')
    
    Block.output('rgb', 'RGB image.')
    
    Block.config('zoom', 'Either 0 for global map, '
                 'or a value giving the size of the window', default=0)

    Block.config('grid', 'Size of the grid (0: turn off)', default=1)
    Block.config('show_sensor_data', 'Show sensor data', default=True)
            
    def update(self):
        f = pylab.figure(frameon=False,
                         figsize=(self.config.width / 100.0,
                                 self.config.width / 100.0))
    
        state = self.input.state
        bounds = state['world']['bounds']
        bx = bounds[0]
        by = bounds[1]
        primitives = state['world']['primitives']
        robot_pose = from_yaml(state['vehicle']['pose'])
        robot_radius = state['vehicle']['radius']
        
        if self.config.grid > 0:
            self.show_grid(bx, by, spacing=self.config.grid, margin=1)
                
        for p in primitives:
            if p['type'] == 'PolyLine':
                points = np.array(p['points']).T
                pylab.plot(points[0, :], points[1, :], 'k-', markersize=10) 
            elif p['type'] == 'Circle':
                facecolor = [0.4, 0.4, 0.4] if p['solid'] else 'none'
                cir = pylab.Circle(p['center'], radius=p['radius'], zorder=1000,
                                   edgecolor=[0.4, 0.4, 0.4], facecolor=facecolor)
                pylab.gca().add_patch(cir)

            else:
                print('Unknown type %r.' % p['type'])

        t, theta = translation_angle_from_SE2(SE2_from_SE3(robot_pose))
        cir = pylab.Circle((t[0], t[1]), radius=robot_radius, zorder=1001,
                           edgecolor='k', facecolor=[0.5, 0.5, 0.5])
        pylab.gca().add_patch(cir)
        pylab.plot([t[0], t[0] + np.cos(theta) * robot_radius * 3],
                   [t[1], t[1] + np.sin(theta) * robot_radius * 3], 'k', zorder=1002)
    
        if self.config.show_sensor_data:
            self.show_sensor_data(pylab, state['vehicle'])
            
        if self.config.zoom == 0:
            bx = bounds[0]
            by = bounds[1]
            m = 0.5
            pylab.axis([bx[0] - m, bx[1] + m, by[0] - m, by[1] + m])
            pylab.axis('equal')
        else:
            m = self.config.zoom
            pylab.axis([t[0] - m, t[0] + m, t[1] - m, t[1] + m])
#            pylab.axis('equal')
            
        if False:
            pylab.plot([0, 2], [0, 0], 'r-')
            pylab.text(2, 0, 'x')
            pylab.plot([0, 0], [0, 2], 'g-')
            pylab.text(0, 2, 'y')
        # turn off ticks labels
        pylab.setp(f.axes[0].get_xticklabels(), visible=False)
        pylab.setp(f.axes[0].get_yticklabels(), visible=False)

 
        self.output.rgb = pylab2rgb(transparent=False, tight=True)

        pylab.close(f.number)

    def show_sensor_data(self, pylab, vehicle):
        robot_pose = from_yaml(vehicle['pose'])
        for attached in vehicle['sensors']:
            sensor_pose = from_yaml(attached['current_pose'])
            sensor_t, sensor_theta = \
                translation_angle_from_SE2(SE2_from_SE3(sensor_pose))
            print('robot: %s' % SE2.friendly(SE2_from_SE3(robot_pose)))
            print(' sens: %s' % SE2.friendly(SE2_from_SE3(sensor_pose)))
            sensor = attached['sensor']
            if sensor['type'] == 'Rangefinder':
                directions = np.array(sensor['directions'])
                observations = attached['current_observations']
                readings = np.array(observations['readings'])
                valid = np.array(observations['valid'])
                directions = directions[valid]
                readings = readings[valid]
                x = []
                y = []
                rho_min = 0.05
                for theta_i, rho_i in zip(directions, readings):
                    x.append(sensor_t[0] + np.cos(sensor_theta + theta_i) * rho_min)
                    y.append(sensor_t[1] + np.sin(sensor_theta + theta_i) * rho_min)
                    x.append(sensor_t[0] + np.cos(sensor_theta + theta_i) * rho_i)
                    y.append(sensor_t[1] + np.sin(sensor_theta + theta_i) * rho_i)
                    x.append(None)
                    y.append(None)
                pylab.plot(x, y, color='y', markersize=0.5, zorder=2000)
            elif sensor['type'] == 'Photoreceptors':
                directions = np.array(sensor['directions'])
                observations = attached['current_observations']
                readings = np.array(observations['readings'])
                luminance = np.array(observations['luminance'])
                valid = np.array(observations['valid'])
                readings[np.logical_not(valid)] = 0.6
                rho_min = 0.5
                for theta_i, rho_i, lum in zip(directions, readings, luminance):
                    x = []
                    y = []
                    x.append(sensor_t[0] + np.cos(sensor_theta + theta_i) * rho_min)
                    y.append(sensor_t[1] + np.sin(sensor_theta + theta_i) * rho_min)
                    x.append(sensor_t[0] + np.cos(sensor_theta + theta_i) * rho_i)
                    y.append(sensor_t[1] + np.sin(sensor_theta + theta_i) * rho_i)
                    pylab.plot(x, y, color=(lum, lum, lum), markersize=0.5, zorder=2000)
            else:
                print('Unknown sensor type %r' % sensor['type'])

    def show_grid(self, bx, by, spacing, margin):
        print('frame')
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
        
