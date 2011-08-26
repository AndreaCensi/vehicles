from . import Vehicle
from ..interfaces import World
from geometry import SE3

class VehicleSimulation():
    
    def __init__(self, vehicle, world):
        assert isinstance(vehicle, Vehicle)
        assert isinstance(world, World)
        
        self.vehicle = vehicle
        self.world = world
        self.timestamp = 0.0
        
    def __repr__(self):
        return 'VSim(%s;%s)' % (self.vehicle, self.world)
            
    def info(self, s):
        print(s) # XXX:
    
    def simulate(self, commands, dt):
        updated = self.world.simulate(dt, self.vehicle)
        self.timestamp += dt
        
        self.vehicle.set_world_primitives(updated)
        if self.vehicle.colliding_pose(self.vehicle.get_pose()).collided:
            self.info('Collision due to dynamic world.')
            self.vehicle_collided = True
        else:
            self.vehicle.simulate(commands, dt)
            self.vehicle_collided = self.vehicle.collision.collided
            if self.vehicle_collided:
                self.info('Collision with object.')
                
    def compute_observations(self):
        observations = self.vehicle.compute_observations()
        
        # TODO: more than one?
        return observations
        
    def new_episode(self):
        self.timestamp = 0.0
        
        max_tries = 100
        for i in range(max_tries): #@UnusedVariable
            episode = self.world.new_episode()
            pose = episode.vehicle_state # TODO: change name
            SE3.belongs(pose)
            self.vehicle.set_world_primitives(self.world.get_primitives())
            collision = self.vehicle.colliding_pose(pose) 
            if not collision.collided:
                self.vehicle.set_pose(pose)
                #print('Setting pose %s' % pose)
                self.id_episode = episode.id_episode    
                return episode
        else:
            raise Exception('Cannot find a non-colliding state.')
        
    def to_yaml(self):
        ''' Returns a YAML-serializable description of the state. '''
        data = {
            'vehicle': self.vehicle.to_yaml(),
            'world': self.world.to_yaml(),
            'timestamp': self.timestamp,
            'id_episode': self.id_episode 
        }
        return data
