from ... import VehicleSimulation, VehiclesConfig, logger
from optparse import OptionParser
from reprep import MIME_PNG, MIME_SVG
import os
from vehicles_cairo import vehicles_has_cairo

usage = """

    %cmd   --vehicle <vehicle> --world <world> [-o where] 
           [-n N] [-z ZOOM] [--figsize INCHES] 
"""


def main():
    if not vehicles_has_cairo:
        logger.error('This program cannot be run if Cairo is not installed.')
        return

    from vehicles_cairo import (vehicles_cairo_display_pdf,
                                vehicles_cairo_display_png,
                                vehicles_cairo_display_svg)

    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()

    parser.add_option("--vehicles", default='*',
                      help="ID vehicle [%default].")
    parser.add_option("--world", default='empty_fixed',
                       help="ID world [%default].")
    parser.add_option("--outdir", "-o",
                      default='vehicles_demo_display_vehicles',
                    help="output directory [%default]")
    parser.add_option("--figsize", default=10, type='float',
                    help="figsize (inches) [%default]")
    parser.add_option("-z", "--zoom", default=1.4, type='float',
                    help="zoom in meters; 0 for full view [%default]")
    parser.add_option("-g", "--grid", default=1, type='float',
                    help="grid size in meters; 0 for no grid [%default]")

    parser.add_option("-d", dest="config", default=".",
                      help="Config directory")

    # TODO: other config dirs

    (options, args) = parser.parse_args()
    if args: raise Exception()

    id_world = options.world

    logger.info('  id_world: %s' % id_world)

    from reprep import Report, MIME_PDF
    basename = 'vehicles_demo'
    r = Report(basename)

    logger.info('Loading configuration from %s' % options.config)
    VehiclesConfig.load(options.config)

    # TODO: selection
    vehicles = VehiclesConfig.vehicles.keys()

    print('Plotting vehicles: %s' % vehicles)

    for id_vehicle in sorted(vehicles):
        sec = r.node(id_vehicle)
        f = sec.figure(cols=3)

        world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
        vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
        simulation = VehicleSimulation(vehicle, world)
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()

        plot_params = dict(grid=options.grid,
                           zoom=options.zoom,
                           width=800, height=800,
                           show_sensor_data=True)

        with f.data_file('start_cairo_png', MIME_PNG) as filename:
            vehicles_cairo_display_png(filename,
                        sim_state=sim_state, **plot_params)

        with f.data_file('start_cairo_pdf', MIME_PDF) as filename:
            vehicles_cairo_display_pdf(filename,
                        sim_state=sim_state, **plot_params)

        with f.data_file('start_cairo_svg', MIME_SVG) as filename:
            vehicles_cairo_display_svg(filename,
                        sim_state=sim_state, **plot_params)


    filename = os.path.join(options.outdir, '%s.html' % basename)
    logger.info('Writing to %r.' % filename)
    r.to_html(filename)




if __name__ == '__main__':
    main()
