from procgraph import Block
from vehicles_cairo import vehicles_cairo_display_all
from procgraph.block_utils import make_sure_dir_exists
import os


class VehiclesCairoDisplay(Block):
    ''' Produces a top-down plot of a circular arena.
    '''

    Block.alias('vehicles_cairo_display')

    Block.config('width', 'Image width in points.', default=800)
    Block.config('height', 'Image height in points.', default=800)
    Block.config('zoom', 'Either 0 for global map, '
                 'or a value giving the size of the window', default=0)

    Block.config('grid', 'Size of the grid (0: turn off)', default=1)
    Block.config('show_sensor_data', 'Show sensor data', default=True)

    Block.config('file', 'Output file (pdf)')

    Block.input('state', 'Simulation state')

    Block.output('rgb', 'RGB image.')

    def init(self):
        import cairo

        self.filename = self.config.file
        self.tmp_filename = self.filename + '.active'

        make_sure_dir_exists(self.filename)
        self.info("Creating PDF %r." % self.filename)

        width = self.config.width
        height = self.config.height

        self.surf = cairo.PDFSurface(self.tmp_filename, #@UndefinedVariable
                                     width, height)

        self.frame = 0

    def update(self):
        self.info('Frame: %s' % self.frame)
        self.frame += 1
        plotting_params = dict(
                    grid=self.config.grid,
                    zoom=self.config.zoom,
                    show_sensor_data=self.config.show_sensor_data)

        width = self.config.width
        height = self.config.height

        sim_state = self.input.state
        import cairo
        # If I don't recreate it, it will crash
        self.cr = cairo.Context(self.surf) #@UndefinedVariable
        vehicles_cairo_display_all(self.cr, width, height,
                                   sim_state, **plotting_params)
        self.surf.flush()
        self.surf.show_page()
        # Free memory self.cr?

    def finish(self):
        self.surf.finish()
        if os.path.exists(self.filename):
            os.unlink(self.filename)
        if os.path.exists(self.tmp_filename):
            os.rename(self.tmp_filename, self.filename)


