from ..interfaces import World
from . import Vehicle

class VehicleSimulation():
    
    def __init__(self, vehicle, world):
        assert isinstance(vehicle, Vehicle)
        assert isinstance(world, World)
        
        self.vehicle = vehicle
        self.world = world
            
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
        for i in range(max_tries):
            episode = self.world.new_episode()
            self.id_episode = episode.id_episode
            self.vehicle.set_world(self.world)
            collision = self.vehicle.colliding_state(episode.vehicle_state) 
            if not collision.collided:
                self.vehicle.set_state(episode.vehicle_state)
                return episode
        else:
            raise Exception('Cannot find a non-colliding state.')
