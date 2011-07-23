from ...configuration import instantiate_spec
from ...interfaces import PolyLine, Circle
from StringIO import StringIO
from contracts import contract
from geometry import translation_angle_from_SE2
from subprocess import Popen, PIPE
import numpy as np
import simplejson #@UnresolvedImport

    # 
    # class CheatDecoder:
    #     def __init__():
    #         self.decoder = simplejson.JSONDecoder()
    #         
    #     def raw_decode(self, buffer):
    #         raise Exception('Unimplemented')
    #         
    #         sys.stderr.write('Parsing buffer of len %d \n' % (len(buffer)))
    #         return Decoder.decoder.raw_decode(buffer)
    #         try: 
    #             obj = cjson.decode(buffer)
    #             print('Succesfully parsed')
    #             return obj, len(buffer)
    #         except Exception as e:
    #             print('not succesfully: %s' % e)
    #             obj, index = Decoder.decoder.raw_decode(buffer)
    #             return obj, index

BVException = Exception

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
            # self.child_stream = JSONStream(self.p.stdout)
            from .cjson_stream import CJSONStream
            self.child_stream = CJSONStream(self.p.stdout)
        except OSError as e:
            #if e.errno == errno.ENOENT:
                msg = ('Could not open connection to raytracer %r: %s.' % 
                       (raytracer, e.strerror))
                raise BVException(msg)
            #raise e
        
        if self.directions is not None:
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

    def set_world_primitives(self, primitives):
        # TODO: make add_circle() add_polyline()
        for x in primitives:
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
                luminance[i] = float('nan')
        
        answer['luminance'] = luminance
        
        return answer
         
        
