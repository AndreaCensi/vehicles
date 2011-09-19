from geometry import SE2_from_SE3, translation_from_SE2, SE3
from nose.plugins.attrib import attr
from vehicles import Box
import numpy as np
from vehicles import VehiclesConfig


@attr('simulation')
def test_collisions():
    L = 10
    world = Box(L, L)
#    id_vehicle = 'd_SE2_rb_v-rf360'
    id_vehicle = 'd_SE2_rb_v-random_5'
    
    vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
    vehicle.set_world_primitives(world.get_primitives())
    vehicle.set_pose(SE3.identity())
    

    commands = np.array([1, 0, 0])
    
    # go straight
    simulation_length = 10
    dt = 0.1
    time = 0
    steps = int(np.ceil(simulation_length / dt))
    for k in range(steps): #@UnusedVariable
        vehicle.simulate(commands, dt)
        pose = SE2_from_SE3(vehicle.get_pose())
        translation = translation_from_SE2(pose)
        #print('t=%.3f %s' % (time, SE2.friendly(pose)))
        time += dt
        assert translation[0] <= 9.5
    assert translation[0] >= 9.4
    assert translation[1] == 0


if __name__ == '__main__':
    test_collisions()
