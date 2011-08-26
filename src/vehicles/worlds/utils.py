from contracts import contract
from vehicles.interfaces.primitives import PolyLine
import numpy as np

@contract(cell_width='>0')
def random_checkerboard(cell_width):
    ''' Utility function to obtain a random checker board. '''
    return ['vehicles.worlds.textures.RandomCheckerboard',
            {'cell_width': cell_width,
             'seed': np.random.randint(100000)}]

def box(id_object, texture, width, length):
    ''' Returns a box. '''
    points = [ [-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
    points = [ (np.array(p) * np.array([width, length])).tolist() for p in points]
    return PolyLine(id_object=0, tags=[],
                    texture=texture, points=points)
