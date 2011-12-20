''' 
    Simple utilities for 2D intersections.

    The normal is returned as a 2d numpy array pointing towards the circle.
    The penetration is a float, >=0 (=0 if touching).
     
'''
from . import contract, np
from contracts import new_contract
from collections import namedtuple


PrimitiveIntersection = namedtuple('PrimitiveIntersection',
                                   'normal penetration')

new_contract('point2', 'seq[2](number)')
new_contract('intersection', 'tuple((array[2],unit_length), >=0)')


@contract(c1='point2', r1='>0', c2='point2', r2='>0', solid2='bool',
          returns='None|intersection')
def circle_circle_intersection(c1, r1, c2, r2, solid2=True):
    ''' The second circle might or might not be solid. '''
    c1 = np.array(c1)
    c2 = np.array(c2)

    dist = np.linalg.norm(c1 - c2)
    if dist > r1 + r2:
        # The two solid circle do not touch
        return None
    else:
        # The two solid circle DO touch
        normal = c1 - c2
        nn = np.linalg.norm(normal)
        if nn > 0:
            normal /= np.linalg.norm(normal)
        else:
            normal = np.array([1, 0])

        if dist < r2 - r1:
            # The first circle is completely inside the second
            if not solid2:
                return None
        penetration = (r1 + r2) - dist
        assert penetration >= 0
        return PrimitiveIntersection(normal, penetration)


@contract(c1='point2', r1='>0', p1='point2', p2='point2',
          returns='None|intersection')
def circle_segment_intersection(c1, r1, p1, p2):
    c1 = np.array(c1)
    p1 = np.array(p1)
    p2 = np.array(p2)
    # projection of c1 onto the line containing p1-p2
    projection, normal, distance = projection_on_line(p=c1, a=p1, b=p2)
    if distance > r1:
        return None
    # check that the projection is inside the segment
    inside = ((projection - p1) * (projection - p2)).sum() <= 0
    if not inside:
        return None

    penetration = r1 - distance
    res = PrimitiveIntersection(normal, penetration)
    return res


@contract(c1='point2', r1='>0', points='seq[>=2](point2)',
          returns='None|intersection')
def circle_polyline_intersection(c1, r1, points):
    closest = None
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]
        collision = circle_segment_intersection(c1, r1, p1, p2)
        if (collision is not None) and (closest is None
                                         or collision.penetration >
                                          closest.penetration):
            closest = collision
    return closest


@contract(p='point2', a='point2', b='point2',
           returns='tuple(array[2], (array[2], unit_length), >=0)')
def projection_on_line(p='point2', a='point2', b='point2'):
    ''' 
        Projects a point onto the line containing the segment a-b. 
        Returns the projected point, the normal pointing to p,
             and the distance of p to the line.
    '''
    t0 = a[0] - b[0]
    t1 = a[1] - b[1]
    one_on_r = 1 / np.sqrt(t0 * t0 + t1 * t1)
    # normal 
    c = t1 * one_on_r
    s = -t0 * one_on_r
    rho = c * a[0] + s * a[1]

    px = c * rho + s * s * p[0] - c * s * p[1]
    py = s * rho - c * s * p[0] + c * c * p[1]

    distance = np.abs(rho - (c * p[0] + s * p[1]))
    proj = np.array([px, py])
    normal = -np.array([c, s]) * np.sign(rho)
    return (proj, normal, distance)
