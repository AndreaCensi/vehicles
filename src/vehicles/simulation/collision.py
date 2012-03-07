from . import circle_polyline_intersection, circle_circle_intersection
from ..interfaces import Circle, PolyLine
from collections import namedtuple

CollisionInfo = namedtuple('CollisionInfo',
                           'collided time primitive normal penetration')


def compute_collision(dynamics_function, max_dt, primitives, radius):
    '''
        dynamics_function: returns the center of the circle at time t.
 
    '''
    # assert that it is ok
    center = dynamics_function(0)
    collision = collides_with(primitives, center, radius)
    assert collision.collided == False, 'We should start from clean state.'

    # first try if all is ok taking a big step
    center = dynamics_function(max_dt)
    collision = collides_with(primitives, center, radius)
    if not collision.collided:
        return CollisionInfo(collided=False, time=max_dt,
                             normal=None, primitive=None, penetration=None)

    # binary search
    def f(t):
        center_t = dynamics_function(t)
        collision = collides_with(primitives, center_t, radius)
        # print('t=%.3f collided %s' % (t, collision.collided))
        return collision.collided

    lower, upper = binary_search(f, 0, max_dt, 0.01)

    #print('Collision between %f and %f' % (lower, upper))

    center = dynamics_function(upper)
    c = collides_with(primitives, center, radius)

    return CollisionInfo(collided=True, time=lower,
                         normal=c.normal, primitive=c.primitive,
                         penetration=c.penetration)

    assert False


def binary_search(f, lower, upper, precision):
    assert f(lower) == False
    assert f(upper) == True
    while upper - lower > precision:
        next_value = upper * 0.5 + lower * 0.5
        if f(next_value):
            upper = next_value
        else:
            lower = next_value
    return lower, upper


def collides_with(primitives, center, radius):
    ''' 
        Checks whether a circle collides with other primitives.
        Returns CollisionInfo. 
    '''
    collisions = []
    for primitive in primitives:
        collides = collides_with_primitive(primitive, center, radius)
        if collides is not None:
            collisions.append((primitive, collides))

    if not collisions:
        return CollisionInfo(collided=False, time=None,
                             normal=None, primitive=None,
                             penetration=None)
    else:
        collisions.sort(key=lambda x: (-x[1].penetration))
        primitive = collisions[0][0]
        normal = collisions[0][1].normal
        penetration = collisions[0][1].penetration

        return CollisionInfo(collided=True, time=None,
                             normal=normal, primitive=primitive,
                             penetration=penetration)


def collides_with_primitive(primitive, center, radius):
    if isinstance(primitive, Circle):
        return circle_circle_intersection(center, radius, primitive.center,
                                          primitive.radius, primitive.solid)
    if isinstance(primitive, PolyLine):
        return circle_polyline_intersection(center, radius, primitive.points)

    # XXX: warn?
