from . import np
from . import (PrimitiveIntersection, circle_circle_intersection,
    circle_segment_intersection)
from geometry import assert_allclose
import unittest


# shortcuts
cci = circle_circle_intersection
csi = circle_segment_intersection


def pi(normal, penetration):
    return PrimitiveIntersection(np.array(normal), penetration)


class CollisionUtilsTest(unittest.TestCase):

    # parameters, result
    all_tests = [
        (cci, ([0, 0], 1, [0, +2.0], 1.0), pi([0, -1], 0.0)),
        (cci, ([0, 0], 1, [0, +1.5], 1.0), pi([0, -1], 0.5)),
        (cci, ([0, 0], 1, [0, -2.0], 1.0), pi([0, +1], 0.0)),
        (cci, ([0, 0], 1, [0, -1.5], 1.0), pi([0, +1], 0.5)),
        (cci, ([0, 0], 1, [0, -1.5], 0.2), None),
        (csi, ([0, 0], 1, [+1.0, +1.0], [+1.0, -1.0]), pi([-1, 0], 0.0)),
        (csi, ([0, 0], 1, [-1.0, +1.0], [-1.0, -1.0]), pi([+1, 0], 0.0)),
        # exchange order of points
        (csi, ([0, 0], 1, [+1.0, -1.0], [+1.0, +1.0]), pi([-1, 0], 0.0)),
        (csi, ([0, 0], 1, [-1.0, -1.0], [-1.0, +1.0]), pi([+1, 0], 0.0)),

        (csi, ([0, 0], 1, [+1.1, +1.0], [+1.1, -1.0]), None),
        (csi, ([0, 0], 1, [+0.5, +1.0], [+0.5, -1.0]), pi([-1, 0], 0.5)),
        (csi, ([0, 0], 1, [-1.0, -1.0], [+1.0, -1.0]), pi([0, +1], 0.0)),
        (csi, ([0, 0], 1, [-1.0, +1.0], [+1.0, +1.0]), pi([0, -1], 0.0)),


        # inside the other
        (cci, ([0, 0], 1, [0, 0], 1.0), pi([1, 0], 2.0)),
        (cci, ([0, 0], 0.5, [0, 0], 1.0), pi([1, 0], 1.5)),

    ]

    def test_circle_circle_intersection(self):
        for function, params, expected in CollisionUtilsTest.all_tests:
            result = function(*params)
            err_msg = 'Invalid match for %s:%s' % (function.__name__,
                                                   params.__repr__())
            err_msg += '\n expected: %s' % expected.__repr__()
            err_msg += '\n obtained: %s' % result.__repr__()
            if expected is None:
                self.assertEqual(expected, result, msg=err_msg)
            else:
                if result is None:
                    raise Exception(err_msg)
                assert_allclose(expected[0], result[0], atol=1e-8,
                                err_msg=err_msg)
                assert_allclose(expected[1], result[1], err_msg=err_msg)


def test_solid():
    assert None != circle_circle_intersection([0, 0], 1, [0, 0], 2, True)
    assert None == circle_circle_intersection([0, 0], 1, [0, 0], 2, False)



