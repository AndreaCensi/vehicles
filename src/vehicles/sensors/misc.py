from . import np
from geometry import assert_allclose


def get_uniform_directions(fov_deg, num_sensels):
    if fov_deg == 360:
        ray_dist = 2 * np.pi / (num_sensels)
        directions = np.linspace(-np.pi + ray_dist / 2,
                                 + np.pi - ray_dist + ray_dist / 2, num_sensels)

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
