from . import contract

# The kinds of objects in our simulation

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

class Source(Primitive):
    def __init__(self, id_object, tags, center, kernel):
        pass    
