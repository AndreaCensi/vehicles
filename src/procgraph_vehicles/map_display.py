from procgraph import Block
from procgraph_mpl import pylab2rgb, pylab
from vehicles.display import display_all


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

        display_all(pylab,
                    state=self.input.state,
                    grid=self.config.grid,
                    zoom=self.config.zoom,
                    show_sensor_data=self.config.show_sensor_data)

        self.output.rgb = pylab2rgb(transparent=False, tight=True)

        pylab.close(f.number)
