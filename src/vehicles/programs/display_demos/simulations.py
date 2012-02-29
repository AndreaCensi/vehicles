from ... import VehicleSimulation, VehiclesConfig, logger
from optparse import OptionParser
from reprep import MIME_PNG, MIME_SVG
import numpy as np
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

    parser.add_option("--vehicle", default='d_SE2_rb_v-rf360',
                      help="ID vehicle [%default].")
    parser.add_option("--world", default='stochastic_box_10',
                       help="ID world [%default].")
    parser.add_option("-n", default=1, type='int',
                    help="number of simulations [%default].")
    parser.add_option("--outdir", "-o", default='display_demo',
                    help="output directory [%default]")
    parser.add_option("--figsize", default=10, type='float',
                    help="figsize (inches) [%default]")
    parser.add_option("-z", "--zoom", default=0, type='float',
                    help="zoom in meters; 0 for full view [%default]")
    parser.add_option("-g", "--grid", default=1, type='float',
                    help="grid size in meters; 0 for no grid [%default]")

    parser.add_option("--cairo", default=False, action='store_true')

    parser.add_option("--seed", default=None, type='int')

    (options, args) = parser.parse_args()
    if args:
        raise Exception()

    id_vehicle = options.vehicle
    id_world = options.world

    logger.info('id_vehicle: %s' % id_vehicle)
    logger.info('  id_world: %s' % id_world)

    if options.seed is None:
        options.seed = np.random.randint(1000000)

    np.random.seed(seed=options.seed)
    logger.info('Using seed %s (your lucky number is %s)' %
                (options.seed, np.random.randint(1000)))

    vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
    world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable

    simulation = VehicleSimulation(vehicle, world)

    from reprep import Report, MIME_PDF
    basename = 'display-%s-%s' % (id_vehicle, id_world)
    r = Report(basename)
    r.text('seed', 'Seed = %s' % options.seed)
    for i in range(options.n):
        sec = r.node('simulation%d' % i)
        f = sec.figure()

        simulation.new_episode()
        simulation.compute_observations()

        sim_state = simulation.to_yaml()

        plot_params = dict(grid=options.grid,
                           zoom=options.zoom, show_sensor_data=True)
#            with f.plot('start', figsize=(options.figsize,
#                                                options.figsize)) as pylab:
#                    display_all(pylab, sim_state, **plot_params)

        with f.data_file('start_cairo_png', MIME_PNG) as filename:
            vehicles_cairo_display_png(filename, width=800, height=800,
                        sim_state=sim_state, **plot_params)

        with f.data_file('start_cairo_pdf', MIME_PDF) as filename:
            vehicles_cairo_display_pdf(filename, width=800, height=800,
                        sim_state=sim_state, **plot_params)

        with f.data_file('start_cairo_svg', MIME_SVG) as filename:
            vehicles_cairo_display_svg(filename, width=800, height=800,
                        sim_state=sim_state, **plot_params)

    filename = os.path.join(options.outdir, 'index.html')
    logger.info('Writing to %r.' % filename)
    r.to_html(filename)


if __name__ == '__main__':
    main()
