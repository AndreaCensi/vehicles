from . import contract, np
from ..interfaces import World
from geometry import pose_from_rotation_translation, random_rotation

class Empty(World):
    
    @contract(bounds='seq[3](seq[2](number))')
    def __init__(self, bounds):
        self.bounds = bounds
        
    def get_primitives(self):
        return []
    
    def _simulate(self, dt):
        return []
        

    def new_episode(self):
        t = np.array([np.random.uniform(b[0], b[1]) for b in self.bounds])
        R = random_rotation() 
        pose = pose_from_rotation_translation(R, t)
        return World.Episode('empty', pose)
         
