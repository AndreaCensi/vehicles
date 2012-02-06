from . import SampledTexture, np, contract
from conf_tools import instantiate_spec
from geometry import assert_allclose
from  scipy.signal import gaussian, convolve #@UnresolvedImport
# TODO: add if/else


class Smoothed(SampledTexture):
    ''' Smooths another texture. '''
    @contract(sigma='>0', resolution='None|>0')
    def __init__(self, sigma, texture, resolution=None):
        sigma = float(sigma)
        if resolution is None:
            resolution = sigma / 10.0

        max_length = 100
        texture = instantiate_spec(texture)

        num_cells = max_length / resolution

        cell_coord = np.linspace(0, max_length, num_cells)
        unsmoothed = texture(cell_coord)

        kernel_size = int(2 * (sigma * 3) / resolution)

        #print('resol: %s' % resolution)
        #print('sigma: %s' % sigma)
        #print('kernel_size: %s' % kernel_size)
        #print kernel

        kernel = gaussian(kernel_size, sigma / resolution)
        kernel = kernel / kernel.sum()

        assert kernel.size == kernel_size
        assert_allclose(kernel.sum(), 1)

        smoothed = convolve(unsmoothed, kernel, 'same')

        assert smoothed.size == cell_coord.size

        SampledTexture.__init__(self, smoothed, resolution)
