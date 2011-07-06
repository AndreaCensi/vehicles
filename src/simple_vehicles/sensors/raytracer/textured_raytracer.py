import errno
from StringIO import StringIO
import simplejson #@UnresolvedImport TODO
from subprocess import Popen, PIPE 

from jsonstream import JSONStream #@UnresolvedImport TODO 

from contracts import contract
from simple_vehicles.interfaces import PolyLine
from geometry import translation_angle_from_SE2
from bootstrapping_olympics.loading import instantiate_spec
from simple_vehicles.interfaces.world_interface import Circle

BVException = Exception
import numpy as np

class Raytracer:
    
    @contract(directions='seq(number)')
    def __init__(self, directions, raytracer='raytracer2'):
        self.raytracer = raytracer
        self.p = None
        self.directions = directions

    def init_connection(self, raytracer):
        try:
            self.p = Popen(raytracer, stdout=PIPE, stdin=PIPE)
            self.child_stream = JSONStream(self.p.stdout)
        except OSError as e:
            if e.errno == errno.ENOENT:
                msg = ('Could not open connection to raytracer %r: %s.' % 
                       (raytracer, e.strerror))
                raise BVException(msg)
            raise e
        
        sensor_setup = { 
            "class": "sensor",
            "directions": list(self.directions)
        }
        self.write_to_connection(sensor_setup)
        
    def write_to_connection(self, object):
        if self.p is None:
            self.init_connection(self.raytracer)
        
        # Make sure that we are sending good json
        sio = StringIO()
        simplejson.dump(object, sio)
        s = sio.getvalue()
        sio2 = StringIO(s)
        simplejson.load(sio2)
        self.p.stdin.write(s)
        self.p.stdin.write('\n') 
        self.p.stdin.flush()


    def read_answer(self, expected):
        answer = self.child_stream.read_next()
        if answer is None:
                raise BVException("Could not communicate with child")
        if ((not isinstance(answer, dict)) or 
            (not 'class' in answer) or
            (answer['class'] != expected)):
            raise Exception('Invalid response %r' % answer) 
        return answer 
    
    def __del__(self):
        if self.p is not None:
            self.p.stdin.close()
            try:
                self.p.terminate()
                #print "Closing pipe %s, %s" % (self.p.stdin, self.p.stdout)
                self.p.wait()
                pass
            except OSError:
                pass
        #print " Closed pipe %s, %s" % (self.p.stdin,self.p.stdout)

    def set_world(self, world, updated=None):
        # TODO: use updated
        for x in world.get_primitives():
            surface = x.id_object
            if isinstance(x, PolyLine):
                msg = { 
                    "class": "add_polyline",
                    "surface": surface,
                    "points": x.points
                }
                self.write_to_connection(msg)
            elif isinstance(x, Circle):
                msg = {
                    "class": "add_circle",
                    'surface': surface,
                    'radius': x.radius,
                    'center': x.center,
                    'solid_inside': 1 if x.solid else 0 
                }
                self.write_to_connection(msg)
            else: 
                raise Exception('Unexpected object %s' % x)

    @contract(pose='SE2')
    def raytracing(self, pose):
        position, orientation = translation_angle_from_SE2(pose)

        query_object = {
            "class": "query_sensor",
            "position": [position[0], position[1]],
            "orientation": orientation
        }
        self.write_to_connection(query_object)
        answer = self.read_answer("query_sensor_response")
        return answer
        
    @contract(center='seq[2](number)', radius='>0')
    def query_circle(self, center, radius):
        """ Returns tuple (hit, surface_id) """
        query_object = {
            "class": "query_circle",
            "center": [ center[0], center[1] ],
            "radius": radius
        }    
         
        self.write_to_connection(query_object)
        answer = self.read_answer("query_circle_response") 
        hit = answer['intersects'] == 1
        if hit:
            surface = answer['surface']
        else:
            surface = None
        
        return hit, surface    
    
        
class TexturedRaytracer(Raytracer):

    def __init__(self, directions, raytracer='raytracer2'):
        Raytracer.__init__(self, directions, raytracer)
        self.surface2texture = {}

    def set_world(self, world, updated=None):
        for x in world.get_primitives():
            surface = x.id_object
            if isinstance(x, PolyLine):
                self.surface2texture[surface] = instantiate_spec(x.texture)
            if isinstance(x, Circle):
                self.surface2texture[surface] = instantiate_spec(x.texture)
        Raytracer.set_world(self, world, updated)
         
    def raytracing(self, pose):
        answer = Raytracer.raytracing(self, pose)
        n = len(answer['surface'])
        luminance = np.zeros(n)
        for i, surface_id in enumerate(answer['surface']):
            if answer['valid'][i]:
                texture = self.surface2texture[surface_id]
                coord = answer['curvilinear_coordinate'][i]
                luminance[i] = texture(coord)
            else:
                luminance[i] = float('nan')
        
        answer['luminance'] = luminance.tolist()
        
        return answer
         
        
