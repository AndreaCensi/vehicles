from . import World, np, contract
from vehicles.configuration.master import VehiclesConfig

__all__ = ['Join']


class Join(World):
    """ Creates a world by joining others. """
    
    @contract(id_worlds='list(str)')
    def __init__(self, id_worlds):
        config = VehiclesConfig
        self.id_worlds = id_worlds
        self.worlds = [config.worlds.instance(id_world) 
                       for id_world in id_worlds] 

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
        return episodes[0] # XXX: should I join?
            
    def simulate(self, dt, vehicle_pose):
        prims = []
        for w in self.worlds:
            prims.extend(w.simulate(dt, vehicle_pose))
        return prims
    
    
