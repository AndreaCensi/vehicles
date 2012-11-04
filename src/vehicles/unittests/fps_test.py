from optparse import OptionParser

from collections import namedtuple
from vehicles import VehiclesConfig, VehicleSimulation
from vehicles.unittests.simulation_tests import random_commands
import time

import numpy as np

import sys


def check_simulation(sim, num_instants, dt):
    sim.new_episode()
    t0 = time.clock()
    count = 0
    commands_spec = sim.vehicle.dynamics.get_commands_spec()

    for i in range(num_instants):  # @UnusedVariable
        cmds = random_commands(commands_spec)
        cmds = np.array([0, 0, 0])
        sim.simulate(cmds, dt)
        sim.compute_observations()
        count += 1
        if count % 10 == 0:
            t1 = time.clock()
            fps = count / (t1 - t0)
            s = ('%s: %.1ffps  (%d %d/%d)%s\r' % 
                             (sim, fps, count, i, num_instants, ' ' * 10))
            sys.stderr.write(s)

        if count % 100 == 0:
            count = 0
            t0 = time.clock()
    print(s)
#    sys.stderr.write('\n')
    #print('%s  fps: %4d  (%d frames)' % (sim.vehicle, fps, count))
    return fps


def fps_main():
    if True:
        import contracts
        contracts.disable_all()

    #import contracts
    #contracts.disable_all()

    usage = """
    #    vehicles = ['d_SE2_rb_v-rf180', 'd_SE2_rb_v-cam180']
#    vehicles = ['d_SE2_rb_v-rf180']
#    vehicles += ['d_SE2_rb_v-cam180']
#    vehicles += ['d_SE2_rb_v-fs_05_12x12']
    id_world = 'box10'
    id_world = 'StocSources_w10_n20_s1'
    
    d_SE2_rb_v-cam_f360_n180_s
     """
    parser = OptionParser(usage=usage)

    parser.add_option("-w", "--world",
                      default='StocSources_w10_n20_s1',
                      help="World")

    parser.add_option("-v", "--vehicle",
                      default=['d_SE2_rb_v-fs_05_12x12'],
                      action='append',
                      help="Vehicles to simulate")
 
    (options, _) = parser.parse_args()
    
    VehiclesConfig.load()

    id_world = options.world
    world = VehiclesConfig.worlds.instance(id_world)  # @UndefinedVariable
    stats = []
    Stat = namedtuple('Stat', 'id_vehicle id_world fps')

    def stat2str(s):
        return "v: %-25s w: %-25s %5dfps" % (s.id_vehicle, s.id_world, s.fps)

#    vehicles = list(VehiclesConfig.vehicles.keys())
#    print vehicles
    
    vehicles = options.vehicle

    T = 200
#    T = 100000
    dt = 0.05
    for id_vehicle in vehicles:
        instance = VehiclesConfig.vehicles.instance  # @UndefinedVariable
        vehicle = instance(id_vehicle)
        print('vehicle: %s' % id_vehicle)
        sim = VehicleSimulation(vehicle, world)
        fps = check_simulation(sim, num_instants=T, dt=dt)
        stats.append(Stat(id_vehicle=id_vehicle, id_world=id_world, fps=fps))
        print(stat2str(stats[-1]))

    print('---- Sorted:')
    stats.sort(key=lambda x: (-x.fps))
    for s in stats:
        print(stat2str(s))

def main():
    profile = True
    #  profile = False
    if not profile:
        fps_main()
    else:
        import cProfile
        cProfile.runctx('fps_test_main()', globals(), locals(), 'fps_prof')
        import pstats
        p = pstats.Stats('fps_prof')
        p.sort_stats('cumulative').print_stats(30)
#        p.print_callers(.5, 'init')
        p.print_callers(.5, 'belongs')
        p.sort_stats('time').print_stats(30)
        

if __name__ == '__main__':
    main()
