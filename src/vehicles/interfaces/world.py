from abc import abstractmethod, ABCMeta
from contracts import contract
from collections import namedtuple
import numpy as np
from geometry import SE2_from_xytheta, SE3_from_SE2
import datetime
 
class World:
    __metaclass__ = ABCMeta
    
    @contract(bounds='seq[3](seq[2](number))')
    def __init__(self, bounds):
        self.bounds = bounds
        
    def __repr__(self):
        return '%s' % self.__class__.__name__
    
    def to_yaml(self):
        return {
            'bounds': self.bounds,
            'primitives': [x.to_yaml() for x in self.get_primitives()]
        }
    
    @abstractmethod
    def get_primitives(self):
        pass
    
    @abstractmethod
    @contract(dt='>0', returns='list(int)')
    def simulate(self, dt, vehicle_pose):
        ''' 
            Simulates the world for dt. 
        
            Returns a list of the updated ids.
        ''' 
        
    Episode = namedtuple('Episode', 'id_episode vehicle_state')

    def new_episode(self):
        ''' Returns an episode tuple. By default it just
            samples a random pose for the robot.
        ''' 
        x = np.random.uniform(self.bounds[0][0], self.bounds[0][1]) 
        y = np.random.uniform(self.bounds[1][0], self.bounds[1][1])
        th = np.random.uniform(0, np.pi * 2)
        vehicle_state = SE3_from_SE2(SE2_from_xytheta([x, y, th]))
        id_episode = unique_timestamp_string()
        return World.Episode(id_episode, vehicle_state)

def unique_timestamp_string():
    now = datetime.datetime.now()
    s = now.isoformat()
    return s.replace('-', '').replace(':', '').replace('T', '_').replace('.', '_')

def isodate():
    now = datetime.datetime.now()
    date = now.isoformat('-')[:19]
    return date


         
