from ... import VehiclesConfig
from .natsort import natsorted
from optparse import OptionParser
import logging
import os

logging.basicConfig()
logger = logging.getLogger("print_config")
logger.setLevel(logging.DEBUG)


usage = """

    This program writes a simple summary of all the VehiclesConfig available.

    vehicles_print_config [-d <config directory>] -o outputdir
"""


def main():
    parser = OptionParser()
    parser.add_option("-o", "--outdir", dest='outdir',
                      help="Output directory")
    parser.add_option("-d", dest="directory",
                      help="base directory for VehiclesConfig", metavar="FILE")
    (options, args) = parser.parse_args()

    if args:
        raise Exception('Spurious arguments')
    if options.outdir is None:
        raise Exception('Please pass --outdir.')

    print_config(options.directory, options.outdir)


def print_config(directory, outdir):
    from reprep import Report

    VehiclesConfig.load(directory)

    def write_report(r):
        out = os.path.join(outdir, '%s.html' % r.nid)
        rd = os.path.join(outdir, 'images')
        logger.info('Writing to %r' % out)
        r.to_html(out, resources_dir=rd)

    worlds = VehiclesConfig.worlds
    r = Report('worlds')
    create_generic_table(r, 'VehiclesConfig', worlds, ['desc', 'code'])
    write_report(r)

    dynamics = VehiclesConfig.dynamics
    r = Report('dynamics')
    create_generic_table(r, 'VehiclesConfig', dynamics, ['desc', 'code'])
    write_report(r)

    sensors = VehiclesConfig.sensors
    r = Report('sensors')
    create_generic_table(r, 'VehiclesConfig', sensors, ['desc', 'code'])
    write_report(r)

    vehicles = VehiclesConfig.vehicles
    r = Report('vehicles')
    create_generic_table(r, 'VehiclesConfig', vehicles,
                          ['desc', 'dynamics', 'id_dynamics', 'sensors'])
    write_report(r)


def create_generic_table(r, nid, name2entry, cols, caption=None):
    names = natsorted(name2entry.keys())
    if names:
        table = []
        found_cols = []
        for name in names:
            c = name2entry[name]
            row = []
            for col in cols:
                if col in c:
                    if not col in found_cols:
                        found_cols.append(col)
                    row.append(c[col])
            table.append(row)
        r.table(nid, table,
                cols=cols, rows=names, caption=caption)
    else:
        logger.warn('warn', 'Empty %r table' % nid)

if __name__ == '__main__':
    main()

