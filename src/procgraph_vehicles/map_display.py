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
            
    def update(self):
        f = pylab.figure(frameon=False,
                        figsize=(self.config.width / 100.0,
                                 self.config.width / 100.0))
    
        state = self.input.state
        bounds = state['world']['bounds']
        primitives = state['world']['primitives']
        robot_pose = from_yaml(state['vehicle']['pose'])
        robot_radius = state['vehicle']['radius']
        
        
        if self.config.grid > 0:
            S = self.config.grid    
            M = 1 # margin (cells)
            bx = bounds[0]
            by = bounds[1]
            params = dict(markersize=0.5, color=[0.8, 0.8, 1], zorder= -1000)
            xmin = (np.floor(bx[0] / S) - M) * S
            xmax = (np.ceil(bx[1] / S) + M) * S
            ymin = (np.floor(by[0] / S) - M) * S
            ymax = (np.ceil(by[1] / S) + M) * S
            for x in np.linspace(xmin, xmax, np.round((xmax - xmin) / S) + 1):
                pylab.plot([x, x], [ymin, ymax], **params)
            for y in np.linspace(ymin, ymax, np.round((ymax - ymin) / S) + 1):
                pylab.plot([xmin, xmax], [y, y], **params)
                
        for p in primitives:
            if p['type'] == 'PolyLine':
                points = np.array(p['points']).T
                pylab.plot(points[0, :], points[1, :], 'k-', markersize=10) 
            elif p['type'] == 'Circle':
                cir = pylab.Circle(p['center'], radius=p['radius'], zorder=1000,
                                   edgecolor=[0.4, 0.4, 0.4], facecolor=[0.4, 0.4, 0.4])
                pylab.gca().add_patch(cir)

            else:
                print('Unknown type %r.' % p['type'])

        t, theta = translation_angle_from_SE2(SE2_from_SE3(robot_pose))
        cir = pylab.Circle((t[0], t[1]), radius=robot_radius, zorder=1001,
                           edgecolor='k', facecolor=[0.5, 0.5, 0.5])
        pylab.gca().add_patch(cir)
        pylab.plot([t[0], t[0] + np.cos(theta) * robot_radius],
                   [t[1], t[1] + np.sin(theta) * robot_radius], 'k', zorder=1002)
    
        for attached in state['vehicle']['sensors']:
            sensor_pose = from_yaml(attached['current_pose'])
            sensor_t, sensor_theta = \
                translation_angle_from_SE2(SE2_from_SE3(sensor_pose))
            print('commands: %s' % state['commands'])
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
                rho_min = 0.5
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
#                print directions
#                print luminance
                readings[np.isnan(readings)] = 0.6
#                print readings
#                directions = directions[valid]
#                readings = readings[valid]
#                luminance = luminance[valid]
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
            
        pylab.plot([0, 2], [0, 0], 'r-')
        pylab.text(2, 0, 'x')
        pylab.plot([0, 0], [0, 2], 'g-')
        pylab.text(0, 2, 'y')
        # turn off ticks labels
        pylab.setp(f.axes[0].get_xticklabels(), visible=False)
        pylab.setp(f.axes[0].get_yticklabels(), visible=False)

 
        self.output.rgb = pylab2rgb(transparent=False, tight=True)

        pylab.close(f.number)
