from . import contract
from .. import cairo_rototranslate
from geometry import SE2, SE2_from_SE3
from vehicles import VehicleSkin, VehiclesConfig


class ComplexSkin(VehicleSkin):
    def __init__(self, skins):
        '''
            skins:
            - skin:
              pose:  (unused)
              joint:
              
        '''
        self.joint_skins = skins

    def njoints_required(self):
        return 1 + max(x['joint'] for x in self.joint_skins)

    @contract(joints='list(tuple(SE3,se3))')
    def draw(self, cr, joints=None, timestamp=None):
        for js in self.joint_skins:
            jointnum = js.get('joint', 0)

            if not(0 <= jointnum < len(joints)):
                msg = ('Invalid joint number #%d. Dynamics has '
                       '%d joints.' % (jointnum, len(joints)))
                raise ValueError(msg)

            skin = js.get('skin')
            pose = js.get('pose', [0, 0, 0]) # TODO: honor this

            skin_impl = VehiclesConfig.specs['skins'].instance(skin)

            robot_pose = SE2_from_SE3(joints[0][0])
            joint_pose = SE2_from_SE3(joints[jointnum][0])
            relative_pose = SE2.multiply(SE2.inverse(robot_pose), joint_pose)

            #print('plotting skin %r at rel pose %r' %
            #      (skin, SE2.friendly(relative_pose)))
            with cairo_rototranslate(cr, relative_pose):
                skin_impl.draw(cr, timestamp=timestamp)
