from . import logger
from optparse import OptionParser
from vehicles import VehiclesConfig
from bootstrapping_olympics.utils import wrap_script_entry_point
import os
import yaml
import itertools

usage = """

    vehicles_create_olympics_config <directory>
    
Creates a "robot" entry for BootstrappingOlympics for each pair
of (vehicle, world) in the Vehicles configuration.
  
"""


def create_olympics_config(pargs):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    # XXX: log
    parser.add_option("-o", dest='outdir',
                      default="./vehicles_olympics_config",
                      help="Output directory [%default].")

    (options, args) = parser.parse_args()

    if args:
        raise Exception('Spurious arguments.')

    if not os.path.exists(options.outdir):
        os.makedirs(options.outdir)
    filename = os.path.join(options.outdir, 'vehicles_olympics.robots.yaml')

    configs = []

    VehiclesConfig.load()

    worlds = VehiclesConfig.worlds.keys()
    vehicles = VehiclesConfig.vehicles.keys()

    logger.info('Found %d worlds, %d vehicles.' % (len(worlds), len(vehicles)))

    for id_world, id_vehicle in itertools.product(worlds, vehicles):
        config = {
          'id': '%s-%s' % (id_vehicle, id_world),
          'desc': 'Vehicle simulation (%s, %s)' % (id_vehicle, id_world),
          'code': [
                   'vehicles_boot.BOVehicleSimulation',
                   {'id_vehicle': id_vehicle,
                    'id_world': id_world,
                    'dt': 0.1 # XXX
                   }]
        }
        configs.append(config)

    with open(filename, 'w') as f:
        yaml.dump(configs, f)

    logger.info('Dumped %d configs to %r.' % (len(configs), filename))


def main():
    wrap_script_entry_point(create_olympics_config, logger)
if __name__ == '__main__':
    main()
