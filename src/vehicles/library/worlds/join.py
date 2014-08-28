
from contracts import contract
from vehicles import World, get_conftools_worlds
import numpy as np

__all__ = ['Join']


class Join(World):
    """ Creates a world by joining others. """
    
    @contract(worlds='list')
    def __init__(self, worlds):
        self.id_worlds = worlds
        self.worlds = [get_conftools_worlds().instance(id_world) 
                       for id_world in worlds] 

        wb = [w.bounds for w in self.worlds]

        bounds = [[np.min([b[0][0] for b in wb]),
                   np.max([b[0][1] for b in wb])],
                  [np.min([b[1][0] for b in wb]),
                   np.max([b[1][1] for b in wb])],
                  [np.min([b[2][0] for b in wb]),
                   np.max([b[2][1] for b in wb])]]
        bounds = np.array(bounds).tolist()
        World.__init__(self, bounds)

    def get_primitives(self):
        prims = []
        for i, w in enumerate(self.worlds):
            wp = w.get_primitives()
            for p in wp:
                p.id_object += i * 1000
            prims.extend(wp)
        return prims
    
    def new_episode(self):
        episodes = [w.new_episode() for w in self.worlds]
        return episodes[0]  # XXX: should I join?
            
    def simulate(self, dt, vehicle_pose):
        prims = []
        for w in self.worlds:
            prims.extend(w.simulate(dt, vehicle_pose))
        return prims
    
    
