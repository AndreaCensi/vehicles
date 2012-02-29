from abc import abstractmethod, ABCMeta
from contracts import contract


class VehicleSkin:
    __metaclass__ = ABCMeta

    @abstractmethod
    @contract(joints='list') # TODO: add what is this
    def draw(self, cr, joints=None, timestamp=None):
        '''
            Draws the vehicle to the given context.
            
            :param cr: Cairo context
            :param joints: list of joints configuration, relative to robot
                coordinates (joints[0] == Identity)
        '''

    def njoints_required(self):
        return 1
