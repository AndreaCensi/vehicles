from ... import VehicleSimulation, VehiclesConfig, logger
from optparse import OptionParser
from reprep import MIME_PNG, MIME_SVG
import os
from ...utils import expand_string

usage = """

    %cmd   [--vehicles <pattern>] --world <world> [-o where] 
           [-n N] [-z ZOOM] [--scale] [--figsize INCHES] 
"""


def main():
    from vehicles_cairo import vehicles_has_cairo
    if not vehicles_has_cairo:
        logger.error('This program cannot be run if Cairo is not installed.')
        return

    from vehicles_cairo import (vehicles_cairo_display_pdf,
                                vehicles_cairo_display_png,
                                vehicles_cairo_display_svg,
                                cairo_plot_circle2)

    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()

    parser.add_option("--vehicles", default='*',
                      help="ID vehicle [%default].")
    parser.add_option("--world", default='SBox2_10a',
                      # default='empty_fixed',
                       help="ID world [%default].")
    parser.add_option("--outdir", "-o",
                      default='vehicles_demo_display_vehicles',
                    help="output directory [%default]")
    parser.add_option("--figsize", default=10, type='float',
                    help="figsize (inches) [%default]")
    parser.add_option("-g", "--grid", default=1, type='float',
                    help="grid size in meters; 0 for no grid [%default]")
    parser.add_option("-d", dest="config", default=".",
                      help="Config directory")
    parser.add_option("--scale", default=False, action='store_true',
                    help="If given, displays the scale with a red circle")

    (options, args) = parser.parse_args()
    if args:
        raise Exception() # XXX

    id_world = options.world

    logger.info('  id_world: %s' % id_world)

    from reprep import Report, MIME_PDF
    basename = 'vehicles_demo'
    r = Report(basename)

    logger.info('Loading configuration from %s' % options.config)
    VehiclesConfig.load(options.config)

    # TODO: selection
    all_vehicles = VehiclesConfig.vehicles.keys()
    if options.vehicles is None:
        vehicles = all_vehicles
    else:
        vehicles = expand_string(options.vehicles, all_vehicles)

    print('Plotting vehicles: %s' % vehicles)

    f0 = r.figure(cols=6)
    f0_data = r.figure(cols=6)

    for id_vehicle in sorted(vehicles):
        sec = r.node(id_vehicle)
        f = sec.figure(cols=6)

        world = VehiclesConfig.specs['worlds'].instance(id_world)
        vehicle = VehiclesConfig.specs['vehicles'].instance(id_vehicle)
        simulation = VehicleSimulation(vehicle, world)
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()

        def draw_scale(cr):
            if options.scale:
                cairo_plot_circle2(cr, 0, 0,
                               vehicle.radius, fill_color=(1, .7, .7))

        plot_params = dict(grid=options.grid,
                           zoom=1.5,
                           zoom_scale_radius=True,
                           width=500, height=500,
                           show_sensor_data=True,
                           show_sensor_data_compact=True,
                           extra_draw_world=draw_scale,
                           bgcolor=None,
                           show_world=False)

        with f.data_file('png_with', MIME_PNG) as filename:
            vehicles_cairo_display_png(filename,
                        sim_state=sim_state, **plot_params)

        f0_data.sub(f.last(), caption=id_vehicle)

        plot_params['show_sensor_data'] = False
        with f.data_file('png_without', MIME_PNG) as filename:
            vehicles_cairo_display_png(filename,
                        sim_state=sim_state, **plot_params)

        f0.sub(f.last(), caption=id_vehicle)

        with f.data_file('svg', MIME_SVG) as filename:
            vehicles_cairo_display_svg(filename,
                        sim_state=sim_state, **plot_params)

        plot_params['grid'] = 0
        plot_params['extra_draw_world'] = None
        with sec.data_file('pdf', MIME_PDF) as filename:
            vehicles_cairo_display_pdf(filename,
                        sim_state=sim_state, **plot_params)

        plot_params['show_robot_body'] = True
        plot_params['show_robot_sensors'] = False
        with f.data_file('png_only_body', MIME_PNG) as filename:
            vehicles_cairo_display_png(filename,
                        sim_state=sim_state, **plot_params)

    filename = os.path.join(options.outdir, 'index.html')
    logger.info('Writing to %r.' % filename)
    r.to_html(filename)


if __name__ == '__main__':
    main()
