from . import Vehicle
from ..interfaces import World
from geometry import SE3
import numpy as np

from . import logger

class VehicleSimulation():
    
    def __init__(self, vehicle, world):
        assert isinstance(vehicle, Vehicle)
        assert isinstance(world, World)
        
        self.vehicle = vehicle
        self.world = world
        self.timestamp = 0.0
      
        self.last_vehicle_observations = None # used for to_yaml()
        
        self.vehicle_collided = None
        cmd_spec = self.vehicle.dynamics.get_commands_spec()
        self.last_commands = np.zeros(len(cmd_spec['format'])) # XXX
        
        self.episode_started = False
        
    def __repr__(self):
        return 'VSim(%s;%s)' % (self.vehicle, self.world)
            
    def info(self, s):
        logger.info(s) # XXX:
    
    def simulate(self, commands, dt):
        if not self.episode_started:
            raise ValueError('Episode not started yet.') # XXX better exception?
        
        commands = np.array(commands)
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
        
        self.last_commands = commands
         
        
    def compute_observations(self):
        if not self.episode_started:
            raise ValueError('Episode not started yet.') # XXX better exception?
        
        observations = self.vehicle.compute_observations()
        self.last_vehicle_observations = observations        
        # TODO: more than one?
        return observations
        
    def new_episode(self):
        self.episode_started = True
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
                return episode 
            #print('Bad random: collision  %s' % str(collision))
        else:
            msg = 'Cannot find a non-colliding state.'
            raise Exception(msg)
        
 
    def to_yaml(self):
        ''' Returns a YAML-serializable description of the state. '''
        if not self.episode_started:
            raise ValueError('Episode not started yet.') # XXX better exception?
        
        # make sure we computed the observations at least once 
        if self.last_vehicle_observations is None:
            self.compute_observations()
            
        data = {
            'version': [1, 0],
            'vehicle': self.vehicle.to_yaml(),
            'world': self.world.to_yaml(),
            'timestamp': self.timestamp,
            'observations':  self.last_vehicle_observations.tolist(),
            'last_commands': self.last_commands.tolist(),
        }
        return data
