from optparse import OptionParser
from reprep import Report
from simple_vehicles.loading import load_configuration, \
    Configuration
import os
import logging

logging.basicConfig();
logger = logging.getLogger("print_config")
logger.setLevel(logging.DEBUG)



usage = """

    This program writes a simple summary of all the configuration available.

    vehicles_print_configuration [-d <config directory>] -o outputdir
""" 


def main():
    parser = OptionParser()
    parser.add_option("-o", "--outdir", dest='outdir',
                      help="Output directory")
    parser.add_option("-d", dest="directory",
                      help="base directory for configuration", metavar="FILE")
    (options, args) = parser.parse_args()

    if args: 
        raise Exception('Spurious arguments')
    if options.outdir is None:
        raise Exception('Please pass --outdir.')
    
    print_configuration(options.directory, options.outdir)
    
def print_configuration(directory, outdir):
    load_configuration(directory)
     
    def write_report(r):
        out = os.path.join(outdir, '%s.html' % r.id)
        rd = os.path.join(outdir, 'images')
        logger.info('Writing to %r' % out)
        r.to_html(out, resources_dir=rd)
        
    worlds = Configuration.worlds
    r = Report('worlds')
    create_generic_table(r, 'configuration', worlds, ['desc', 'code'])
    write_report(r)
    
    dynamics = Configuration.dynamics
    r = Report('dynamics')
    create_generic_table(r, 'configuration', dynamics, ['desc', 'code'])
    write_report(r)
    
    sensors = Configuration.sensors
    r = Report('sensors')
    create_generic_table(r, 'configuration', sensors, ['desc', 'code'])
    write_report(r)
    
    vehicles = Configuration.vehicles
    r = Report('vehicles')
    create_generic_table(r, 'configuration', vehicles, ['desc', 'dynamics', 'sensors'])
    write_report(r)
    

def create_generic_table(r, nid, name2entry, cols, caption=None):
    names = list(name2entry.keys())
    if names:
        table = []
        for name in names:
            c = name2entry[name]
            row = []
            for col in cols:
                row.append(c[col])
            table.append(row)
        r.table(nid, table,
                cols=cols, rows=names, caption=caption)
    else: 
        logger.warn('warn', 'Empty %r table' % nid)    
    
if __name__ == '__main__':
    main()

