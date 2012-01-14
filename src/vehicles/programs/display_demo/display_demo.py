from ... import logger
from optparse import OptionParser
from vehicles import VehiclesConfig
from vehicles.display import display_all
from vehicles.simulation import VehicleSimulation
import os
from vehicles_cairo.display_all import vehicles_cairo_display_all
from reprep.constants import MIME_PNG

usage = """

    %cmd   --vehicle <vehicle> --world <world> [-o where] 
           [-n N] [-z ZOOM] [--figsize INCHES] 
"""


def main():
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

    (options, args) = parser.parse_args()
    if args: raise Exception()

    id_vehicle = options.vehicle
    id_world = options.world

    logger.info('id_vehicle: %s' % id_vehicle)
    logger.info('  id_world: %s' % id_world)

    vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
    world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
    simulation = VehicleSimulation(vehicle, world)

    from reprep import Report, MIME_PDF
    basename = 'display-%s-%s' % (id_vehicle, id_world)
    r = Report(basename)

    for i in range(options.n):
        sec = r.node('simulation%d' % i)
        f = sec.figure()
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()
        if True:
            print('Not cairo')
            with f.plot('start', figsize=(options.figsize,
                                                options.figsize)) as pylab:
                    display_all(pylab, sim_state, grid=options.grid, zoom=options.zoom,
                                show_sensor_data=True)

            if False:
                with f.plot('pdf', mime=MIME_PDF,
                                  figsize=(options.figsize,
                                          options.figsize)) as pylab:
                        display_all(pylab, sim_state, grid=1, zoom=0, show_sensor_data=True)
#        else:
#            print('Cairo')
            with f.data_file('start_cairo', MIME_PNG) as filename:
#                filename_pdf = filename + '.pdf'
                vehicles_cairo_display_all(filename,
                            sim_state, grid=options.grid, zoom=options.zoom,
                             show_sensor_data=True)


    filename = os.path.join(options.outdir, '%s.html' % basename)
    logger.info('Writing to %r.' % filename)
    r.to_html(filename)


if __name__ == '__main__':
    main()
