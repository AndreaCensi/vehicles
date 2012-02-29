from ... import logger
from conf_tools import instantiate_spec
from optparse import OptionParser
from vehicles import GeometricShape, VehiclesConfig
import numpy as np
import os
from pprint import pformat

usage = """
    
    vehicles_inspect_textures -w <id_world> [-n <N>]  -o <outdir>

"""


def inspect_textures():
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()

    parser.add_option("-w", "--world", default='stochastic_box_10',
                       help="ID world [%default].")

    parser.add_option("-n", default=1, type='int',
                    help="number of simulations [%default].")
    parser.add_option("--outdir", "-o", default='out-inspect_textures',
                    help="output directory [%default]")
    parser.add_option("--figsize", default=1, type='float',
                    help="figsize (inches) [%default]")
    parser.add_option("-z", "--zoom", default=0, type='float',
                    help="zoom in meters; 0 for full view [%default]")

    (options, args) = parser.parse_args()
    if args:
        raise Exception() # XXX

    id_world = options.world

    logger.info('  id_world: %s' % id_world)

    world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable

    from reprep import Report
    basename = 'inspect_textures-%s' % (id_world)
    r = Report(basename)

    for i in range(options.n):
        sec = r.node('simulation%d' % i)
        world.new_episode()
        primitives = world.get_primitives()
        for p in primitives:
            psec = sec.node('%s_%d' % (p.__class__.__name__, p.id_object))
            psec.text('object', pformat(p.to_yaml()))

            f = psec.figure(cols=2)
            if not isinstance(p, GeometricShape):
                continue

            perimeter = p.get_perimeter()
            texture = instantiate_spec(p.texture)

            chunk_size = 10
            nchunks = np.ceil(perimeter * 1.0 / chunk_size)

            for c in range(int(nchunks)):
                xfrom = c * chunk_size
                xto = np.minimum(perimeter, xfrom + chunk_size)
                N = 1000
                x = np.linspace(xfrom, xto, N)

                lum = texture(x)

                with f.plot('chunk%s' % c,
                              figsize=(options.figsize * 10,
                                      options.figsize)) as pylab:
                    pylab.plot(x, lum)
                    pylab.axis((xfrom, xfrom + chunk_size, -0.1, 1.1))

#
#        if False:
#            with f.data_pylab('pdf', mime=MIME_PDF,
#                              figsize=(options.figsize,
#                                      options.figsize)) as pylab:
#                    display_all(pylab, sim_state, grid=1, zoom=0, 
# show_sensor_data = True)
#            f.last().add_to(f)

    filename = os.path.join(options.outdir, 'index.html')
    logger.info('Writing to %r.' % filename)
    r.to_html(filename)


if __name__ == '__main__':
    inspect_textures()
