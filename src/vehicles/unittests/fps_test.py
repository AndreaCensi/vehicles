from collections import namedtuple
from vehicles import VehiclesConfig, VehicleSimulation
from vehicles.unittests.simulation_tests import random_commands
import time


def check_simulation(sim, num_instants, dt):
    sim.new_episode()
    t0 = time.clock()
    count = 0
    for i in range(num_instants):  # @UnusedVariable
        cmds = random_commands(sim.vehicle.commands_spec)
        sim.simulate(cmds, dt)
        sim.compute_observations()
        count += 1
#        if count % 10 == 0:
#            sys.stderr.write('.')
#    sys.stderr.write('\n')
    t1 = time.clock()
    fps = count / (t1 - t0)
    #print('%s  fps: %4d  (%d frames)' % (sim.vehicle, fps, count))
    return fps


def main():
    import contracts
    contracts.disable_all()

    VehiclesConfig.load()

    id_world = 'box10'
    world = VehiclesConfig.worlds.instance(id_world)  # @UndefinedVariable
    stats = []
    Stat = namedtuple('Stat', 'id_vehicle id_world fps')

    def stat2str(s):
        return "v: %-25s w: %-25s %5dfps" % (s.id_vehicle, s.id_world, s.fps)

    vehicles = list(VehiclesConfig.vehicles.keys())
    print vehicles
    vehicles = ['d_SE2_rb_v-rf180', 'd_SE2_rb_v-cam180']
    vehicles = ['d_SE2_rb_v-rf180']
    T = 200
    dt = 0.05
    for id_vehicle in vehicles:
        instance = VehiclesConfig.vehicles.instance  # @UndefinedVariable
        vehicle = instance(id_vehicle)
        print('vehicle: %s' % vehicle)
        sim = VehicleSimulation(vehicle, world)
        fps = check_simulation(sim, num_instants=T, dt=dt)
        stats.append(Stat(id_vehicle=id_vehicle, id_world=id_world, fps=fps))
        print(stat2str(stats[-1]))

    print('---- Sorted:')
    stats.sort(key=lambda x:-x.fps)
    for s in stats:
        print(stat2str(s))

    pass

if __name__ == '__main__':
    profile = True
    profile = False
    if not profile:
        main()
    else:
        import cProfile
        cProfile.run('main()', 'fps_prof')
        import pstats
        p = pstats.Stats('fps_prof')
        p.sort_stats('cumulative').print_stats(10)
        p.sort_stats('time').print_stats(10)
