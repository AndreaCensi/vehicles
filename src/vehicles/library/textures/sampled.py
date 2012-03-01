from . import Texture, contract, np


class SampledTexture(Texture):

    @contract(values='array', cell_width='>0')
    def __init__(self, values, cell_width):
        self.values = values
        self.cell_width = cell_width

    def __call__(self, t):
        s = np.round(np.floor(t / self.cell_width))
        s = s.astype('int')
        s = s % self.values.size
        return self.values[s]
