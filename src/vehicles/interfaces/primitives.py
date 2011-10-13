''' These are the geometric primitives in our simulation that define
    the world. '''

from . import contract
from abc import abstractmethod
from conf_tools import  instantiate_spec
import numpy as np

class Primitive:
    def __init__(self, id_object, tags):
        self.id_object = id_object
        self.tags = tags

    def __repr__(self):
        return self.to_yaml().__repr__()
    


class PolyLine(Primitive):
    @contract(id_object='int', tags='seq(str)', texture='x', # XXX
              points='list[>0](seq[2](number))')
    def __init__(self, id_object, tags, texture, points):
        Primitive.__init__(self, id_object, tags)
        self.texture = texture
        self.points = points
        
    def to_yaml(self):
        return {'type': 'PolyLine',
                'surface': self.id_object,
                'tags': self.tags,
                'texture': self.texture,
                'points': self.points}


class Circle(Primitive):
    @contract(id_object='int', tags='seq(str)', texture='x', # XXX
              center='seq[2](number)', radius='>0', solid='bool')
    def __init__(self, id_object, tags, texture, center, radius, solid=False):
        Primitive.__init__(self, id_object, tags)
        self.texture = texture
        self.radius = radius
        self.solid = solid
        self.center = [0, 0]
        self.set_center(center)
        
    @contract(center='seq[2](number)')
    def set_center(self, center):
        self.center[0] = float(center[0])
        self.center[1] = float(center[1])
    
    def to_yaml(self):
        return {'type': 'Circle',
                'surface': self.id_object,
                'tags': self.tags,
                'texture': self.texture,
                'center': self.center,
                'radius': self.radius,
                'solid': self.solid}

class Field:
    
    @abstractmethod
    def get_intensity_value(self, point):
        ''' Returns the intensity at the given point. '''
    
    @contract(X='array[HxW]', Y='array[HxW]', returns='array[HxW]')
    def get_intensity_values(self, X, Y):
        ''' Returns the intensity values at a series of points. '''
        

class Source(Primitive, Field):
    ''' A point-source is what field samplers are sensitive to. '''
    
    @contract(center='seq[2](number)') 
    def __init__(self, id_object, tags, center, kernel_spec):
        '''
            :param:center: 2D position of the source
            :param:kernel: Scalar function from distance to intensity.
                           Described as a code spec.
        ''' 
        Primitive.__init__(self, id_object, tags)
        self.kernel_spec = kernel_spec
        self.kernel = instantiate_spec(self.kernel_spec)
        self.set_center(center)
    
    def set_center(self, center):
        self.center = np.array(center)
        
    def to_yaml(self):
        return {'type': 'Source',
                'id_object': self.id_object,
                'tags': self.tags,
                'center': self.center.tolist(),
                'kernel_spec': self.kernel_spec}

    @staticmethod
    def from_yaml(s):
        return Source(s['id_object'], s['tags'],
                      s['center'], s['kernel_spec'])

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
        
        
        
        
