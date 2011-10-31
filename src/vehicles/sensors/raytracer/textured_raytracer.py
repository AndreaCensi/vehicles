from . import CJSONStream, np
from ...interfaces import PolyLine, Circle
from conf_tools import instantiate_spec
from contracts import contract
from geometry import translation_angle_from_SE2
from subprocess import Popen, PIPE
import cjson


class Raytracer:
    
    @contract(directions='None|seq(number)')
    def __init__(self, directions=None, raytracer='raytracer2'):
        ''' 
            directions are needed to call raytrace; 
            not necessary to call query_circle. 
        '''
        self.raytracer = raytracer
        self.p = None
        self.directions = directions

    def init_connection(self, raytracer):
        try:
            self.p = Popen(raytracer, stdout=PIPE, stdin=PIPE)
            self.child_stream = CJSONStream(self.p.stdout)
        except OSError as e:
            msg = ('Could not open connection to raytracer %r: %s.' % 
                   (raytracer, e.strerror))
            raise Exception(msg)
        
        if self.directions is not None:
            sensor_setup = { 
                "class": "sensor",
                "directions": list(self.directions)
            }
            self.write_to_connection(sensor_setup)
        
    def write_to_connection(self, ob):
        if self.p is None:
            self.init_connection(self.raytracer)
        self.p.stdin.write(cjson.encode(ob))
        self.p.stdin.write('\n') 
        self.p.stdin.flush()


    def read_answer(self, expected):
        answer = self.child_stream.read_next()
        if answer is None:
                raise Exception("Could not communicate with child")
        if ((not isinstance(answer, dict)) or 
            (not 'class' in answer) or
            (answer['class'] != expected)):
            raise Exception('Invalid response %r' % answer) 
        return answer 
    
    def __del__(self):
        if not 'p' in self.__dict__:
            # partial initialization
            return
        if self.p is None:
            return
        
        self.p.stdin.close()
        try:
            self.p.terminate()
            #print "Closing pipe %s, %s" % (self.p.stdin, self.p.stdout)
            self.p.wait()
            pass
        except OSError:
            pass
        #print " Closed pipe %s, %s" % (self.p.stdin,self.p.stdout)

    @contract(surface_id='int', center='seq[2](number)', radius='>0', solid='bool')
    def add_circle(self, surface_id, center, radius, solid):
        msg = {
            "class": "add_circle",
            'surface': surface_id,
            'radius': radius,
            'center': center,
            'solid_inside': 1 if solid else 0 
        }
        self.write_to_connection(msg)


    @contract(surface_id='int', points='list(seq[2](number))')        
    def add_polyline(self, surface_id, points):
        msg = { 
            "class": "add_polyline",
            "surface": surface_id,
            "points": points
        }
        self.write_to_connection(msg)

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
        
        # We have null -> None; make sure that None -> nan
        for field in list(answer.keys()):
            value = answer[field]
            if isinstance(value, list):
                value = [ x if x is not None else np.nan for x in value]
                value = np.array(value)
                answer[field] = value
        
        answer['valid'] = answer['valid'].astype('bool')
        
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

    # XXX: move somewhere else?    
    def set_world_primitives(self, primitives):
        for x in primitives:
            if isinstance(x, PolyLine):
                self.add_polyline(x.id_object, x.points)
            elif isinstance(x, Circle):
                self.add_circle(x.id_object, x.center, x.radius, x.solid)


        
class TexturedRaytracer(Raytracer):

    def __init__(self, directions, raytracer='raytracer2'):
        Raytracer.__init__(self, directions, raytracer)
        self.surface2texture = {}

    def set_world_primitives(self, primitives):
        # XXX: inefficient
        for x in primitives:
            surface = x.id_object
            if isinstance(x, PolyLine):
                self.surface2texture[surface] = instantiate_spec(x.texture)
            if isinstance(x, Circle):
                self.surface2texture[surface] = instantiate_spec(x.texture)
        Raytracer.set_world_primitives(self, primitives)
         
    @contract(pose='SE2')
    def raytracing(self, pose):
        answer = Raytracer.raytracing(self, pose)
        
        n = len(answer['surface'])
        luminance = np.zeros(n)
        for i, surface_id in enumerate(answer['surface']):
            if answer['valid'][i]:
                if not surface_id in self.surface2texture:
                    raise Exception("Unknown surface %r; I know %r." % 
                                    (surface_id, self.surface2texture.keys()))
                texture = self.surface2texture[surface_id]
                coord = answer['curvilinear_coordinate'][i]
                # TODO: make this more efficient
                luminance[i] = texture(coord)
            else:
                luminance[i] = np.NaN
        
        answer['luminance'] = luminance
        
        return answer
         
        
