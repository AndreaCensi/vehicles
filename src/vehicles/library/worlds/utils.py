from . import PolyLine, contract, np


@contract(cell_width='>0')
def random_checkerboard(cell_width):
    ''' Utility function to obtain a random checker board. '''
    texture = ['vehicles.library.textures.RandomCheckerboard',
               dict(cell_width=cell_width, seed=np.random.randint(100000))]
    return texture


@contract(cell_width='>0', sigma='>0')
def random_checkerboard_smooth(cell_width, sigma):
    ''' Utility function to obtain a smoothed random checker board. '''
    texture = ['vehicles.library.textures.RandomCheckerboard',
               dict(cell_width=cell_width, seed=np.random.randint(100000))]
    return ['vehicles.library.textures.Smoothed',
            dict(sigma=sigma, texture=texture)]


def blackwhite_checkerboard(cell_width):
    texture = ['vehicles.library.textures.BWCheckerboard',
               dict(cell_width=cell_width)]
    return texture


def box(id_object, texture, width, length):
    ''' Returns a box. '''
    points = [[-1, -1], [-1, 1], [1, 1], [1, -1], [-1, -1]]
    points = [(np.array(p) * np.array([width, length])).tolist()
              for p in points]
    return PolyLine(id_object=id_object, tags=[],
                    texture=texture, points=points)


class Counter:

    def __init__(self, start=0):
        self.k = start - 1

    def __call__(self):
        self.k += 1
        return self.k

