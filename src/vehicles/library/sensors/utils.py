from . import np, contract
from geometry import assert_allclose

__all__ = ['get_random_directions', 'get_uniform_directions']


@contract(fov_deg='>0', num_sensels='N,>1', returns='array[N](>=-pi,<=pi)')
def get_random_directions(fov_deg, num_sensels):
    """ Returns a random disposition of the sensels. """
    f = np.deg2rad(fov_deg) / 2
    return np.random.uniform(-f, +f, num_sensels)


@contract(fov_deg='>0', num_sensels='N,>1', returns='array[N](>=-pi,<=pi)')
def get_uniform_directions(fov_deg, num_sensels):
    """ Returns a set of directions uniform in space """
    if fov_deg == 360:
        ray_dist = 2 * np.pi / (num_sensels)
        directions = np.linspace(-np.pi + ray_dist / 2,
                                 + np.pi - ray_dist + ray_dist / 2,
                                 num_sensels)

        assert_allclose(directions[-1] - directions[0], 2 * np.pi - ray_dist)

        t = np.rad2deg(directions)
        a = t[1:] - t[:-1]
        b = t[0] - t[-1] + 360
        
        
        assert_allclose(a[0], b)

    else:
        fov_rad = np.radians(fov_deg)
        directions = np.linspace(-fov_rad / 2, +fov_rad / 2, num_sensels)

        assert_allclose(directions[-1] - directions[0], fov_rad)
    assert len(directions) == num_sensels
    return directions
