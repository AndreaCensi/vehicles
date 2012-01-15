from . import contract, np
from ..interfaces import PolyLine


@contract(cell_width='>0')
def random_checkerboard(cell_width):
    ''' Utility function to obtain a random checker board. '''
    texture = ['vehicles.worlds.textures.RandomCheckerboard',
               dict(cell_width=cell_width, seed=np.random.randint(100000))]
    return texture


@contract(cell_width='>0', sigma='>0')
def random_checkerboard_smooth(cell_width, sigma):
    ''' Utility function to obtain a smoothed random checker board. '''
    texture = ['vehicles.worlds.textures.RandomCheckerboard',
               dict(cell_width=cell_width, seed=np.random.randint(100000))]
    return ['vehicles.worlds.textures.Smoothed',
            dict(sigma=sigma, texture=texture)]


def box(id_object, texture, width, length):
    ''' Returns a box. '''
    points = [[-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
    points = [(np.array(p) * np.array([width, length])).tolist()
              for p in points]
    return PolyLine(id_object=0, tags=[],
                    texture=texture, points=points)

class Counter:
    def __init__(self):
        self.k = -1
    def __call__(self):
        self.k += 1
        return self.k

