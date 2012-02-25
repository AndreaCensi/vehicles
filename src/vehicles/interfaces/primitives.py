'''
    These are the geometric primitives in our simulation that define
    the world.
'''

# TODO: remove "tags"

from . import contract, np
from .. import VehiclesConstants
from abc import abstractmethod
from conf_tools import instantiate_spec


class Primitive:
    def __init__(self, id_object, tags):
        self.id_object = id_object
        self.tags = tags

    def __repr__(self):
        return self.to_yaml().__repr__()

    type2class = {}

    @staticmethod
    def from_yaml(s):
        t = s['type']
        if not t in Primitive.type2class:
            msg = ('Could not find type %r (I know %s).' %
                   (t, Primitive.type2class.keys()))
            raise ValueError(msg)
        return Primitive.type2class[t].from_yaml(s)


class GeometricShape:

    def __init__(self, texture):
        ''' The texture is a code spec. '''
        self.texture = texture

    @abstractmethod
    def get_perimeter(self):
        ''' Returns the total perimeter of this shape. '''
        pass


class PolyLine(Primitive, GeometricShape):
    @contract(id_object='int', tags='seq(str)',
              points='list[>0](seq[2](number))')
    def __init__(self, id_object, tags, texture, points):
        Primitive.__init__(self, id_object, tags)
        GeometricShape.__init__(self, texture)

        self.points = points

    def get_perimeter(self):
        t = 0
        for i in range(len(self.points) - 1):
            p1 = np.array(self.points[i])
            p2 = np.array(self.points[i + 1])
            t += np.linalg.norm(p1 - p2)
        return t

    def to_yaml(self):
        return {'type': VehiclesConstants.PRIMITIVE_POLYLINE,
                'surface': self.id_object,
                'tags': self.tags,
                'texture': self.texture,
                'points': self.points}

    @staticmethod
    def from_yaml(s):
        assert s['type'] == VehiclesConstants.PRIMITIVE_POLYLINE
        return PolyLine(id_object=s.get('surface', None),
                        tags=s.get('tags', []),
                        texture=s.get('texture', None), # XXX
                        points=s.get('points'))


class Circle(Primitive, GeometricShape):

    @contract(id_object='int', tags='seq(str)',
              center='seq[2](number)', radius='>0', solid='bool')
    def __init__(self, id_object, tags, texture, center, radius, solid=False):
        Primitive.__init__(self, id_object, tags)
        GeometricShape.__init__(self, texture)

        self.radius = radius
        self.solid = solid
        self.center = [0, 0]
        self.set_center(center)

    def get_perimeter(self):
        return 2 * np.pi * self.radius

    @contract(center='seq[2](number)')
    def set_center(self, center):
        self.center[0] = float(center[0])
        self.center[1] = float(center[1])

    def to_yaml(self):
        return {'type': VehiclesConstants.PRIMITIVE_CIRCLE,
                'surface': self.id_object,
                'tags': self.tags,
                'texture': self.texture,
                'center': self.center,
                'radius': self.radius,
                'solid': self.solid}

    @staticmethod
    def from_yaml(s):
        assert s['type'] == VehiclesConstants.PRIMITIVE_CIRCLE
        return Circle(id_object=s.get('surface', None),
                        tags=s.get('tags', []),
                        texture=s.get('texture', None), # XXX
                        center=s.get('center'),
                        radius=s.get('radius'),
                        solid=s.get('solid', False))


class Field:

    @abstractmethod
    def get_intensity_value(self, point):
        ''' Returns the intensity at the given point. '''

    @contract(X='array[HxW]', Y='array[HxW]', returns='array[HxW]')
    def get_intensity_values(self, X, Y):
        ''' Returns the intensity values at a series of points. '''


class Source(Primitive, Field):
    ''' 
        A point-source is what field samplers are sensitive to. 
    '''

    @contract(center='seq[2](number)')
    def __init__(self, id_object, tags, center, kernel_spec):
        '''
            Initializes the structure.
            
            :param center: 2D position of the source
            :param kernel: Scalar function from distance to intensity.
                           Described as a code spec.
        '''
        Primitive.__init__(self, id_object, tags)
        self.kernel_spec = kernel_spec
        self.kernel = instantiate_spec(self.kernel_spec)
        self.set_center(center)

    def set_center(self, center):
        self.center = np.array(center)

    @contract(point='seq[2](number)')
    def get_intensity_value(self, point):
        distance = np.linalg.norm(np.array(point) - self.center)
        return self.kernel(distance)

    @contract(X='array[HxW]', Y='array[HxW]', returns='array[HxW]')
    def get_intensity_values(self, X, Y):
        xc = X - self.center[0]
        yc = Y - self.center[1]
        D = np.hypot(xc, yc)
        C = self.kernel(D)
        return C

    def to_yaml(self):
        return {'type': VehiclesConstants.PRIMITIVE_SOURCE,
                'id_object': self.id_object,
                'tags': self.tags,
                'center': self.center.tolist(),
                'kernel_spec': self.kernel_spec}

    @staticmethod
    def from_yaml(s):
        assert s['type'] == VehiclesConstants.PRIMITIVE_SOURCE
        return Source(id_object=s.get('surface', None),
                      tags=s.get('tags', []),
                      center=s['center'],
                      kernel_spec=s['kernel_spec'])


Primitive.type2class[VehiclesConstants.PRIMITIVE_POLYLINE] = PolyLine
Primitive.type2class[VehiclesConstants.PRIMITIVE_CIRCLE] = Circle
Primitive.type2class[VehiclesConstants.PRIMITIVE_SOURCE] = Source
