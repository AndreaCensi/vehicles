from contracts import contract
from procgraph import Block
from procgraph.block_utils import make_sure_dir_exists
from vehicles_cairo import cairo_transform, vehicles_cairo_display_all
import itertools
import numpy as np
import os
from vehicles_cairo.utils import cairo_set_color, cairo_save
from procgraph_images.copied_from_reprep import posneg, scale
from procgraph_pil.pil_operations import resize


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

    Block.config('display_sidebar', default=True)
    Block.config('sidebar_width', default=200)

    Block.input('boot_obs', '')
    Block.input('state', 'Simulation state')
    Block.input('observations', 'Observations')
    Block.input('commands', 'Commands')

    Block.output('rgb', 'RGB image.')

    def init(self):
        import cairo

        self.filename = self.config.file
        self.tmp_filename = self.filename + '.active'

        make_sure_dir_exists(self.filename)
        self.info("Creating PDF %r." % self.filename)

        total_width = self.config.width
        if self.config.display_sidebar:
            total_width += self.config.sidebar_width

        self.surf = cairo.PDFSurface(self.tmp_filename, #@UndefinedVariable
                                     total_width,
                                     self.config.height)

        self.frame = 0

    def update(self):
        plotting_params = dict(
                    grid=self.config.grid,
                    zoom=self.config.zoom,
                    show_sensor_data=self.config.show_sensor_data)

        boot_obs = self.input.boot_obs

        id_episode = boot_obs['id_episode'].item()
        id_vehicle = boot_obs['id_robot'].item()
        sim_state = self.input.state
        observations = self.input.observations
        commands = self.input.commands

        #timestamp = boot_obs['timestamp'].item()
        timestamp = boot_obs['time_from_episode_start'].item()

        import cairo
        # If I don't recreate it, it will crash
        cr = cairo.Context(self.surf) #@UndefinedVariable

        with cairo_save(cr):
            if self.config.display_sidebar:
                padding = 0.03 * self.config.width
                map_width = self.config.width - 2 * padding
                map_height = self.config.height - 2 * padding
                cr.translate(padding, padding)
            else:
                map_width = self.config.width
                map_height = self.config.height

            vehicles_cairo_display_all(cr,
                                       map_width,
                                       map_height,
                                       sim_state, **plotting_params)
            if self.config.display_sidebar:
                cr.set_line_width(1)
                cr.set_source_rgb(0, 0, 0)
                cr.rectangle(0, 0, map_width, map_height)
                cr.stroke()

        if self.config.display_sidebar:
            with cairo_transform(cr, t=[self.config.width, 0]):
                create_sidebar(cr, width=self.config.sidebar_width,
                               height=self.config.height,
                               sim_state=sim_state,
                               id_vehicle=id_vehicle,
                               id_episode=id_episode,
                               timestamp=timestamp,
                               observations=observations, commands=commands)

        self.surf.flush()
        self.surf.show_page() # Free memory self.cr?

    def finish(self):
        self.surf.finish()
        if os.path.exists(self.filename):
            os.unlink(self.filename)
        if os.path.exists(self.tmp_filename):
            os.rename(self.tmp_filename, self.filename)
        self.info("Completed %r." % self.filename)


def create_sidebar(cr, width, height, sim_state, id_vehicle, id_episode,
                   timestamp, observations, commands):
#    M = 10
#    cr.rectangle(M, M, width - M, height - M)
#    cr.set_line_width(1)
#    cr.set_source_rgb(1, 0, 0)
#    cr.stroke()

    #M = 12
    M = width / 20.0
    label_font = 'Mono'
    legend_font = 'Serif'
    legend_font_size = M * 0.75
    details_font_size = width / 30.0

    cr.set_source_rgb(0, 0, 0)

    padding_fraction = 0.1
    padding = width * padding_fraction
    nvalues = 128
    bar_width = 0.4 * width
    bar_ratio = 0.15
    bar_height = bar_width * bar_ratio
    spacer = 0.05 * width

    values = np.linspace(-1, +1, nvalues)
    values = np.vstack([values] * 1)
    colorbar_posneg = posneg(values)
    values = np.linspace(-1, +1, nvalues)
    values = np.vstack([values] * 1)
    colorbar_scale = scale(values)

    cr.translate(0, 2 * M)
    with cairo_transform(cr, t=[width / 2, 0]):
        cr.select_font_face(label_font)
        cr.set_font_size(M)
        cairo_text_centered(cr, 'observations')
    cr.translate(0, M * 0.8)

    with cairo_transform(cr, t=[padding, 0]):
        data_width = width - 2 * padding
        last_height = cairo_pixels(cr, observations, data_width)

    cr.translate(0, last_height)

    cr.translate(0, spacer)

    with cairo_transform(cr, t=[width / 2, 0]):
        with cairo_transform(cr, t=[-bar_width / 2, 0]):
            last_height = cairo_pixels(cr, colorbar_scale,
                         bar_width, height=bar_height,
                          grid_color=None)

        cr.set_font_size(legend_font_size)
        cr.select_font_face(legend_font)

        with cairo_transform(cr, t=[0, bar_height / 2]):
            with cairo_transform(cr, t=[-bar_width / 2 - M / 2, 0]):
                cairo_text_align(cr, '0', 'right', 'middle')
            with cairo_transform(cr, t=[+bar_width / 2 + M / 2, 0]):
                cairo_text_align(cr, '1', 'left', 'middle')

    cr.translate(0, last_height + spacer * 3)

    with cairo_transform(cr, t=[width / 2, 0]):
        cr.select_font_face(label_font)
        cr.set_font_size(M)
        cairo_text_centered(cr, 'commands')
    cr.translate(0, M * 0.8)

    padding = padding * 2
    with cairo_transform(cr, t=[padding, 0]):
        data_width = width - 2 * padding
        last_height = cairo_pixels(cr, commands, data_width)
    cr.translate(0, last_height)

    cr.translate(0, spacer)

    with cairo_transform(cr, t=[width / 2, 0]):
        with cairo_transform(cr, t=[-bar_width / 2, 0]):
            last_height = cairo_pixels(cr, colorbar_posneg,
                         bar_width, height=bar_width * bar_ratio,
                          grid_color=None)

        cr.set_font_size(legend_font_size)
        cr.select_font_face(legend_font)

        with cairo_transform(cr, t=[0, bar_height / 2]):
            with cairo_transform(cr, t=[-bar_width / 2 - M / 2, 0]):
                cairo_text_align(cr, '-1', 'right', 'middle')
            with cairo_transform(cr, t=[+bar_width / 2 + M / 2, 0]):
                cairo_text_align(cr, '+1', 'left', 'middle')

    cr.translate(0, last_height + spacer * 2)

    cr.translate(width / 10, 0)
    strings = ['vehicle: %s' % id_vehicle,
               'episode: %s' % id_episode,
               '   time: %6.2f' % timestamp]
    cr.select_font_face('Mono')
    cr.set_font_size(details_font_size)
    line = details_font_size * 1.2
    for s in strings:
        with cairo_save(cr):
            cr.show_text(s)
            cr.stroke()
        cr.translate(0, line)


def cairo_text_align(cr, text, halign='left', valign='bottom'):
    extents = cr.text_extents(text)
    width = extents[2]
    height = extents[3]
    if halign == 'center':
        x = -width / 2.0
    elif halign == 'left':
        x = 0
    elif halign == 'right':
        x = -width
    else:
        assert False

    if valign == 'middle':
        y = +height / 2.0
    elif valign == 'bottom':
        y = 0
    elif valign == 'top':
        y = +height
    else:
        assert False

    t = [x, y]
    with cairo_transform(cr, t=t):
        cr.show_text(text)
        cr.stroke()


def cairo_text_centered(cr, text):
    extents = cr.text_extents(text)
    width = extents[2]
    with cairo_transform(cr, t=[-width / 2.0, 0]):
        cr.show_text(text)
        cr.stroke()


@contract(x='array[HxWx3](uint8)',
          width='>0')
def cairo_pixels(cr, x, width, height=None, grid_color=[1, .9, .9],
                 border_color=[0, 0, 0]):
    x = np.transpose(x, [1, 0, 2])

    pw = width * 1.0 / x.shape[0]
    if height is None:
        ph = pw
        height = pw * x.shape[1]
    else:
        ph = height * 1.0 / x.shape[1]

    bleed = 0.5
    # not sure j, i
    for i, j in itertools.product(range(x.shape[0]),
                                  range(x.shape[1])):
        with cairo_transform(cr, t=[i * pw,
                                    j * ph]):
            cr.rectangle(0, 0, pw + bleed, ph + bleed)
            col = x[i, j, :] / 255.0
            cr.set_source_rgb(col[0], col[1], col[2])
            cr.fill()

    if grid_color is not None:
        cairo_set_color(cr, grid_color)
        for i, j in itertools.product(range(x.shape[0]),
                                      range(x.shape[1])):
            with cairo_transform(cr, t=[i * pw,
                                        j * ph]):
                cr.set_line_width(1)
                cr.rectangle(0, 0, pw, ph)
                cr.stroke()

    cr.rectangle(0, 0, width, height)
    cr.set_line_width(1)
    cairo_set_color(cr, border_color)
    cr.stroke()

    return height


