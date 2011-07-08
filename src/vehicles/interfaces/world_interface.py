from abc import abstractmethod, ABCMeta
from contracts import contract
from collections import namedtuple

class World:
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def get_primitives(self):
        pass
    
    @contract(dt='>0', returns='list(int)')
    def simulate(self, dt):
        ''' 
            Simulates the world for dt. 
        
            Returns a list of the updated ids.
        '''
        updated = self._simulate(dt)
        return updated
    
    @abstractmethod
    def _simulate(self, dt):
        pass
        
        
    Episode = namedtuple('Episode', 'id_episode vehicle_state')

    @abstractmethod
    def new_episode(self):
        ''' Returns an episode tuple. '''
         
