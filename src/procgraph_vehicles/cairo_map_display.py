from contracts import contract
from procgraph import Block
from procgraph.block_utils import make_sure_dir_exists
from procgraph_images import posneg, scale, reshape2d
from vehicles_cairo import (cairo_set_color, cairo_save, cairo_transform,
    vehicles_cairo_display_all)
import itertools
import numpy as np
import os
from procgraph import BadConfig


class VehiclesCairoDisplay(Block):
    ''' Produces a top-down plot of a circular arena.
    '''

    Block.alias('vehicles_cairo_display')

    Block.config('format', 'pdf|png', default='pdf')
    Block.config('file', 'Output file (pdf)', default=None)

    Block.config('width', 'Image width in points.', default=768)
    Block.config('height', 'Image height in points.', default=768)

    # Sidebar options
    Block.config('display_sidebar', default=True)
    Block.config('sidebar_width', default=1024 - 768)

    Block.config('trace', 'Trace the path', default=False)

    Block.config('zoom', 'Either 0 for global map, '
                 'or a value giving the size of the window', default=0)

    Block.config('grid', 'Size of the grid (0: turn off)', default=1)
    Block.config('show_sensor_data', 'Show sensor data', default=True)

    Block.input('boot_obs', '')

    Block.output('rgb', 'RGB data (png)')

    def init(self):

        self.format = self.config.format

        self.total_width = self.config.width
        if self.config.display_sidebar:
            self.total_width += self.config.sidebar_width

        self.frame = 0

        if self.format == 'pdf':
            self.init_pdf()
        elif self.format == 'png':
            self.init_png()
        else:
            raise BadConfig('Invalid format %r.' % self.format, self, 'format')

    def init_pdf(self):
        self.filename = self.config.file
        self.tmp_filename = self.filename + '.active'
        make_sure_dir_exists(self.filename)
        self.info("Creating file %r." % self.filename)
        import cairo
        self.surf = cairo.PDFSurface(self.tmp_filename, #@UndefinedVariable
                                     self.total_width,
                                     self.config.height)

    def init_png(self):
        import cairo
        w, h = self.total_width, self.config.height
        # note (w,h) here and (h,w,h*4) below; I'm not sure but it works
        self.argb_data = np.empty((h, w, 4), dtype=np.uint8)
        self.argb_data.fill(255)

        self.surf = cairo.ImageSurface.create_for_data(#@UndefinedVariable
                        self.argb_data,
                        cairo.FORMAT_ARGB32, #@UndefinedVariable
                         w, h, w * 4)

    def update(self):
        if self.format == 'pdf':
            self.update_pdf()
        elif self.format == 'png':
            self.update_png()
        else:
            assert False

    def update_png(self):
        import cairo
        # If I don't recreate it, it will crash
        cr = cairo.Context(self.surf) #@UndefinedVariable
        self.draw_everything(cr)
        self.surf.flush()

        self.output.rgb = self.argb_data[:, :, :3].copy()

    def update_pdf(self):
        import cairo
        # If I don't recreate it, it will crash
        cr = cairo.Context(self.surf) #@UndefinedVariable
        self.draw_everything(cr)
        self.surf.flush()
        self.surf.show_page() # Free memory self.cr?

    def draw_everything(self, cr):
        plotting_params = dict(
                    grid=self.config.grid,
                    zoom=self.config.zoom,
                    show_sensor_data=self.config.show_sensor_data)

        boot_obs = self.input.boot_obs

        id_episode = boot_obs['id_episode'].item()
        id_vehicle = boot_obs['id_robot'].item()
        sim_state = boot_obs['extra'].item()['robot_state']
        observations = scale(reshape2d(boot_obs['observations']), min_value=0,
                             nan_color=[1, 1, 1])
        commands = posneg(reshape2d(boot_obs['commands']), max_value=(+1),
                              nan_color=[1, 1, 1])
        commands_source = boot_obs['commands_source'].item()
        #timestamp = boot_obs['timestamp'].item()
        timestamp = boot_obs['time_from_episode_start'].item()
        #id_vehicle = str(boot_obs['commands'])

        with cairo_save(cr):
            if self.config.display_sidebar:
                padding = 0.03 * self.config.width
                map_width = self.config.width - 2 * padding
                map_height = self.config.height - 2 * padding
                cr.translate(padding, padding)
            else:
                map_width = self.config.width
                map_height = self.config.height

            with cairo_save(cr):
                cr.rectangle(0, 0, map_width, map_height)
                cr.clip()

                # TODO: implement trace

                vehicles_cairo_display_all(cr,
                                       map_width,
                                       map_height,
                                       sim_state,
                                       **plotting_params)

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
                               observations=observations,
                               commands=commands,
                               commands_source=commands_source)

    def finish(self):
        if self.format == 'pdf':
            self.finish_pdf()

    def finish_pdf(self):
        self.surf.finish()
        if os.path.exists(self.filename):
            os.unlink(self.filename)
        if os.path.exists(self.tmp_filename):
            os.rename(self.tmp_filename, self.filename)
        self.info("Completed %r." % self.filename)


def create_sidebar(cr, width, height, sim_state, id_vehicle, id_episode,
                   timestamp, observations, commands, commands_source):
    import cairo
    fo = cairo.FontOptions() #@UndefinedVariable
    fo.set_hint_style(cairo.HINT_STYLE_FULL) #@UndefinedVariable
    fo.set_antialias(cairo.ANTIALIAS_GRAY) #@UndefinedVariable
    cr.set_font_options(fo)

    #M = width / 20.0
    M = width / 15.0

    legend_font_size = M * 0.75
    details_font_size = M

    label_font = 'Mono'
    legend_font = 'Serif'

    cr.set_source_rgb(0, 0, 0)

    padding_fraction = 0.1
    padding = width * padding_fraction
    nvalues = 128
    bar_width = 0.4 * width
    bar_ratio = 0.15
    bar_height = bar_width * bar_ratio
    spacer = 0.05 * width

    values = np.linspace(-1, +1, nvalues)
    values = np.vstack([values] * 1).T
    colorbar_posneg = posneg(values)
    values = np.linspace(-1, +1, nvalues)
    values = np.vstack([values] * 1).T
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
               '  agent: %s' % commands_source,
               'episode: %s' % id_episode,
               '   time: %6.2f' % timestamp,
               ]
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
    #x = np.transpose(x, [1, 0, 2])

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


