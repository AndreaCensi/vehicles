from vehicles import PolyLine, Circle
from contracts import contract
from geometry import translation_angle_from_SE2
import raytracer


class MyRaytracer(raytracer.Raytracer):

    @contract(pose='SE2')
    def raytracing(self, pose):
        position, orientation = translation_angle_from_SE2(pose)
        return self.query_sensor(position, orientation)

    def set_world_primitives(self, primitives):
        for x in primitives:
            if isinstance(x, PolyLine):
                self.add_polyline(x.id_object, x.points)
            elif isinstance(x, Circle):
                self.add_circle(x.id_object, x.center, x.radius, x.solid)

