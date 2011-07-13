from . import Vehicle
from ..interfaces import World
from geometry import SE3

class VehicleSimulation():
    
    def __init__(self, vehicle, world):
        assert isinstance(vehicle, Vehicle)
        assert isinstance(world, World)
        
        self.vehicle = vehicle
        self.world = world
        
    def __repr__(self):
        return 'VSim(%s;%s)' % (self.vehicle, self.world)
            
    def simulate(self, commands, dt):
        self.vehicle.simulate(commands, dt)
        updated = self.world.simulate(dt)
        if len(updated) > 0: 
            self.vehicle.set_world(self.world, updated)
                
    def compute_observations(self):
        observations = self.vehicle.compute_observations()
        
        # TODO: more than one?
        return observations
        
    def new_episode(self):
        max_tries = 100
        for i in range(max_tries): #@UnusedVariable
            episode = self.world.new_episode()
            self.id_episode = episode.id_episode
            pose = episode.vehicle_state # TODO: change name
            SE3.belongs(pose)
            self.vehicle.set_world(self.world)
            collision = self.vehicle.colliding_pose(pose) 
            if not collision.collided:
                self.vehicle.set_pose(pose)
                return episode
        else:
            raise Exception('Cannot find a non-colliding state.')
