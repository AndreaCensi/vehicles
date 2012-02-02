from . import SampledTexture, np, contract


class RandomCheckerboard(SampledTexture):

    @contract(num='int,>=1', cell_width='>0')
    def __init__(self, cell_width=1, num=100, seed=0):
        generator = np.random.mtrand.RandomState(seed)
        values = generator.uniform(0, 1, num)

        SampledTexture.__init__(self, values, cell_width)


class BWCheckerboard(SampledTexture):

    @contract(num='int,>=1', cell_width='>0')
    def __init__(self, cell_width=1, num=100):
        values = np.zeros(num)
        for i in range(num):
            values[i] = i % 2
        SampledTexture.__init__(self, values, cell_width)
