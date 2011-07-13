from vehicles.unittests.simulation_tests import random_commands
from vehicles.configuration.load_all import load_vehicles_config, VehiclesConfig
import itertools
from vehicles.configuration.instance_all import instance_world, instance_vehicle
from vehicles.simulation.simulation import VehicleSimulation
import time
import sys
import contracts


def check_simulation(sim, num_instants, dt):
    sim.new_episode()
    t0 = time.clock()
    count = 0
    for i in range(num_instants): #@UnusedVariable
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
    contracts.disable_all()
    load_vehicles_config()
        
    world = instance_world('empty')
    stats = []
    for  id_vehicle in VehiclesConfig.vehicles:
        vehicle = instance_vehicle(id_vehicle)
        sim = VehicleSimulation(vehicle, world) 
        fps = check_simulation(sim, num_instants=200, dt=0.05)
        stats.append((sim, fps))
        s = stats[-1]
        print('%5dfps %30s' % (s[1], s[0]))

    print('---- Sorted:')    
    stats.sort(key=lambda x:-x[1])
    for s in stats:
        print('%5dfps %30s' % (s[1], s[0]))
    
                    
    pass
    
if __name__ == '__main__':
    if True:
        main()
    else:
        import cProfile
        cProfile.run('main()', 'fps_prof')
        import pstats
        p = pstats.Stats('fps_prof')
        p.sort_stats('cumulative').print_stats(10)
        p.sort_stats('time').print_stats(10)
