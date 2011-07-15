from ..interfaces import World

class Empty(World):
    
    def get_primitives(self):
        return []
    
    def simulate(self, dt, vehicle_pose):
        return []    

  
