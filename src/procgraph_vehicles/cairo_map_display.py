from cairo_utils import cairo_pixels, cairo_text_align
from geometry import SE2_from_SE3, SE3
from procgraph import BadConfig, Block
from procgraph.block_utils import make_sure_dir_exists
from procgraph_images import posneg, scale, reshape2d
from vehicles_cairo import (cairo_save, cairo_transform,
    vehicles_cairo_display_all, cairo_rototranslate, cairo_ref_frame)
import numpy as np
import os
import subprocess


class VehiclesCairoDisplay(Block):
    ''' Produces a top-down plot of a circular arena. '''

    Block.alias('vehicles_cairo_display')

    Block.config('format', 'pdf|png', default='pdf')
    Block.config('file', 'Output file (pdf)', default=None)
    Block.output('rgb', 'RGB data (png)')
    Block.config('transparent', 'Outputs RGB with transparent bg',
                 default=False)

    Block.config('width', 'Image width in points.', default=600)
    Block.config('height', 'Image height in points.', default=600)
    Block.config('sidebar_width', default=200)

    # Sidebar options
    Block.config('display_sidebar', default=True)
    Block.config('trace', 'Trace the path', default=False)
    Block.config('plotting_params',
                 'Configuration to pass to vehicles_cairo_display_all()',
                 default={})
    Block.config('sidebar_params',
                 'Configuration to pass to create_sidebar()',
                 default={})

    Block.config('swf', 'Converts PDF to SWF using pdf2swf', default=True)

    Block.input('boot_obs', '')

    def get_shape(self):
        w = self.config.width
        if self.config.display_sidebar:
            w += self.config.sidebar_width
        h = self.config.height
        
        return (w, h) 

    def init(self):
        self.format = self.config.format
        (w, h) = self.get_shape()
        self.total_width = w
        self.total_height = h
        self.frame = 0

        if self.format == 'pdf':
            self.init_pdf()
        elif self.format == 'png':
            self.init_png()
        else:
            raise BadConfig('Invalid format %r.' % self.format, self, 'format')
        self.count = 0
        self.fps = None
        self.t0 = None

        self.tmp_cr = None

    def init_pdf(self):
        self.filename = self.config.file
        self.tmp_filename = self.filename + '.active'
        make_sure_dir_exists(self.filename)
        self.info("Creating file %r." % self.filename)
        import cairo
        self.surf = cairo.PDFSurface(self.tmp_filename,  # @UndefinedVariable
                                     self.total_width,
                                     self.total_height)

    def init_png(self):
        import cairo
        w, h = self.total_width, self.total_height
        # note (w,h) here and (h,w,h*4) below; I'm not sure but it works
        self.argb_data = np.empty((h, w, 4), dtype=np.uint8)
        self.argb_data.fill(255)

        self.surf = cairo.ImageSurface.create_for_data(# @UndefinedVariable
                        self.argb_data,
                        cairo.FORMAT_ARGB32,  # @UndefinedVariable
                         w, h, w * 4)

    def update(self):
        # Estimate fps
        if self.count == 0:
            self.t0 = self.get_input_timestamp(0)
        if self.count >= 1:
            delta = self.get_input_timestamp(0) - self.t0
            self.fps = 1.0 * self.count / delta

        self.count += 1

        if self.format == 'pdf':
            self.update_pdf()
        elif self.format == 'png':
            self.update_png()
        else:
            assert False

    def update_png(self):
        import cairo

        cr = cairo.Context(self.surf)  # @UndefinedVariable

        self.draw_everything(cr)
        self.surf.flush()

        if not self.config.transparent:
            rgb = self.argb_data[:, :, :3].copy()
            # fix red/blue inversion
            rgb[:, :, 0] = self.argb_data[:, :, 2]
            rgb[:, :, 2] = self.argb_data[:, :, 0]
            assert rgb.shape[2] == 3
        else:
            rgb = self.argb_data.copy()
            # fix red/blue inversion
            rgb[:, :, 0] = self.argb_data[:, :, 2]
            rgb[:, :, 2] = self.argb_data[:, :, 0]
            assert rgb.shape[2] == 4

        self.output.rgb = rgb

    def update_pdf(self):
        import cairo
        # If I don't recreate it, it will crash
        cr = cairo.Context(self.surf)  # @UndefinedVariable
        if not self.config.transparent:
            # Set white background
            bg_color = [1, 1, 1]
            cr.rectangle(0, 0, self.total_width, self.total_height)
            cr.set_source_rgb(bg_color[0], bg_color[1], bg_color[2])
            cr.fill()
        else:
            # Green screen :-)
            cr.rectangle(0, 0, self.total_width, self.total_height)
            cr.set_source_rgba(0, 1, 0, 0)
            cr.fill()
        self.draw_everything(cr)
        self.surf.flush()
        self.surf.show_page()  # Free memory self.cr?

    def draw_everything(self, cr):

        boot_obs = self.input.boot_obs

        if 'id_episode' in boot_obs:
            id_episode = boot_obs['id_episode'].item()
        else:
            id_episode = ''
            
        id_vehicle = boot_obs['id_robot'].item()
        
        if 'extra' in boot_obs:
            extra = boot_obs['extra'].item()
        else:
            extra = {}

        def extra_draw_world(cr):
            if 'servonav' in extra:
                plot_servonave(cr, extra['servonav'])

            if 'servoing_poses' in extra:
                plot_servoing_poses(cr, extra['servoing_poses'])

        plotting_params = self.config.plotting_params

        plotting_params['extra_draw_world'] = extra_draw_world

        sidebar_params = self.config.sidebar_params

        # todo: check
        sim_state = extra['robot_state']

        observations_values = boot_obs['observations']

        commands = boot_obs['commands']

        commands_source = boot_obs['commands_source'].item()
        timestamp = boot_obs['time_from_episode_start'].item()

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
                               observations_values=observations_values,
                               commands_values=commands,
                               commands_source=commands_source,
                               **sidebar_params)

    def finish(self):
        if self.format == 'pdf':
            self.finish_pdf()

    def finish_pdf(self):
        self.surf.finish()
        if os.path.exists(self.filename):
            os.unlink(self.filename)
        if os.path.exists(self.tmp_filename):
            os.rename(self.tmp_filename, self.filename)

        if self.config.swf:
            if self.fps is None:
                self.error('Only one frame seen?')
            else:
                basename, _ = os.path.splitext(self.filename)
                swf = '%s.swf' % basename
                try:
                    command = ['pdf2swf',
                                           # "-b", # --defaultviewer
                                           # "-l", # --defaultloader
                                           '-G',  # flatten
                                           '-s', 'framerate=%d' % self.fps,
                                            self.filename,
                                            '-o', swf]
                    self.info(' '.join(command))
                    subprocess.check_call(command)
                except Exception as e:
                    self.error('Could not convert to swf: %s' % e)
                    if os.path.exists(swf):
                        os.unlink(swf)

        self.info("Completed %r." % self.filename)


class VehiclesDisplay(VehiclesCairoDisplay):
    ''' Produces a top-down plot of a circular arena. '''

    Block.alias('vehicles_cairo_display_all')

    Block.config('format', 'pdf|png', default='pdf')
    Block.config('file', 'Output file (pdf)', default=None)
    Block.output('rgb', 'RGB data (png)')
    Block.config('transparent', 'Outputs RGB with transparent bg',
                 default=False)

    Block.config('width', 'Image width in points.', default=600)
    Block.config('height', 'Image height in points.', default=600)
    Block.config('trace', 'Trace the path', default=False)
    Block.config('plotting_params',
                 'Configuration to pass to vehicles_cairo_display_all()',
                 default={})
    Block.config('swf', 'Converts PDF to SWF using pdf2swf', default=True)

    Block.input('boot_obs')
    
    def get_shape(self):
        w = self.config.width
        h = self.config.height
        return (w, h)

    def draw_everything(self, cr):
        sim_state = self.input.boot_obs
        map_width = self.config.width
        map_height = self.config.height
        plotting_params = self.config.plotting_params
        with cairo_save(cr):
            cr.rectangle(0, 0, map_width, map_height)
            cr.clip()

            # TODO: implement trace

            vehicles_cairo_display_all(cr,
                                   map_width,
                                   map_height,
                                   sim_state,
                                   **plotting_params)



def create_sidebar(cr, width, height, sim_state, id_vehicle, id_episode,  # @UnusedVariable
                   timestamp, observations_values,
                   commands_values, commands_source,
                   bg_color=None,
                   show_observations=True,
                   show_commands=True,
                   show_annotations=True):

    if len(commands_values.shape) == 1:
        commands_values = np.array([commands_values.tolist()])

    commands_rgb = posneg(commands_values,
                          max_value=(+1),  # not sure +1 
                          nan_color=[1, 1, 1])

    observations_rgb = scale(reshape2d(observations_values), min_value=0,
                         nan_color=[1, 1, 1])

    import cairo
    if bg_color is not None:
        cr.rectangle(0, 0, width, height)
        cr.set_source_rgb(bg_color[0], bg_color[1], bg_color[2])
        cr.fill()

    fo = cairo.FontOptions()  # @UndefinedVariable
    fo.set_hint_style(cairo.HINT_STYLE_FULL)  # @UndefinedVariable
    fo.set_antialias(cairo.ANTIALIAS_GRAY)  # @UndefinedVariable
    cr.set_font_options(fo)

    # M = width / 20.0
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
    values = np.vstack([values] * 1)
    colorbar_posneg = posneg(values)
    values = np.linspace(-1, +1, nvalues)
    values = np.vstack([values] * 1)
    colorbar_scale = scale(values)

    cr.translate(0, 2 * M)

    if show_observations:
        with cairo_transform(cr, t=[width / 2, 0]):
            cr.select_font_face(label_font)
            cr.set_font_size(M)
            cairo_text_align(cr, 'observations', halign='center')

        cr.translate(0, M * 0.8)

        with cairo_transform(cr, t=[padding, 0]):
            data_width = width - 2 * padding
            # Don't draw grid if there are many pixels
            if max(observations_rgb.shape[0], observations_rgb.shape[1]) > 15:
                grid_color = None
            else:
                grid_color = [1, .9, .9]

            last_height = cairo_pixels(cr, observations_rgb, width=data_width,
                                       # Force square
                                       height=data_width,
                                       grid_color=grid_color)

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

    if show_commands:
        with cairo_transform(cr, t=[width / 2, 0]):
            cr.select_font_face(label_font)
            cr.set_font_size(M)
            cairo_text_align(cr, 'commands', halign='center')
        cr.translate(0, M * 0.8)

        padding = padding * 2
        with cairo_transform(cr, t=[padding, 0]):
            data_width = width - 2 * padding
            last_height = cairo_pixels(cr, commands_rgb, data_width)
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

    if show_annotations:
        cr.translate(width / 10, 0)
        strings = ['vehicle: %s' % id_vehicle,
                   '  agent: %s' % commands_source,
                   'episode: %s' % id_episode,
                   '   time: %6.2f' % timestamp,
                   ]
        cr.select_font_face('Mono')

        max_len = max(len(x) for x in strings)
        padding = 5
        font_size = 1.6 * width / (max_len + padding)
        cr.set_font_size(font_size)
        line = details_font_size * 1.2
        for s in strings:
            with cairo_save(cr):
                cr.show_text(s)
                cr.stroke()
            cr.translate(0, line)

 
def plot_servoing_poses(cr, servoing_poses):
    # TODO
    goal = SE3.from_yaml(servoing_poses['goal'])
    with cairo_rototranslate(cr, goal):
        cairo_ref_frame(cr, l=0.5)


def plot_servonave(cr, servonav):
    locations = servonav['locations']
#    current_goal = servonav['current_goal']
    for _, loc in enumerate(locations):
        pose = SE2_from_SE3(SE3.from_yaml(loc['pose']))
        with cairo_rototranslate(cr, pose):
#            if current_goal == i:
#                cairo_ref_frame(cr, l=0.5)
#            else:
            grey = [.6, .6, .6]
            cairo_ref_frame(cr, l=0.5, x_color=grey, y_color=grey)


