from . import MyRaytracer, np
from vehicles import PolyLine, Circle
from conf_tools import instantiate_spec
from contracts import contract
import warnings


class TexturedRaytracer(MyRaytracer):

    def __init__(self, directions, raytracer='raytracer2'):
        MyRaytracer.__init__(self, directions, raytracer)
        self.surface2texture = {}

    def set_world_primitives(self, primitives):
        # XXX: inefficient
        for x in primitives:
            surface = x.id_object
            if isinstance(x, PolyLine):
                self.surface2texture[surface] = instantiate_spec(x.texture)
            if isinstance(x, Circle):
                self.surface2texture[surface] = instantiate_spec(x.texture)
        MyRaytracer.set_world_primitives(self, primitives)

    @contract(pose='SE2')
    def raytracing(self, pose):
        answer = MyRaytracer.raytracing(self, pose)

        #c0 = time.clock()
        surface = answer['surface']
        valid = answer['valid']
        n = len(surface)
        luminance = np.zeros(n)
        # TODO: make this step vectorial

        if np.min(answer['curvilinear_coordinate']) < 0:
            msg = ('coords in [%s, %s] but expect positive' %
                    (np.min(answer['curvilinear_coordinate']),
                     np.max(answer['curvilinear_coordinate'])))
            warnings.warn(msg)

        luminance.fill(np.nan)
        for surface_id in  np.unique(surface):
            if not surface_id in self.surface2texture:
                continue
            texture = self.surface2texture[surface_id]
            which = (surface == surface_id) & valid
            if np.sum(which) == 0:
                continue
            coord = answer['curvilinear_coordinate'][which]
            luminance[which] = texture(coord)

        #print('texturized in %sms' % ((time.clock() - c0) * 1000))
        answer['luminance'] = luminance

        return answer
