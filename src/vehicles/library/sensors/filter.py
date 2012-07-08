#from . import contract, np
#from vehicles import VehicleSensor

#
#class Filter(VehicleSensor):
#
#    def __init__(self, id_sensor, ):
#        spec = {
#            'desc': 'Random sensor',
#            'shape': [num_sensels],
#            'format': 'C',
#            'range': [0, +1],
#            'extra': {}
#        }
#        VehicleSensor.__init__(self, spec)
#        self.num_sensels = num_sensels
#
#    def to_yaml(self):
#        return {'type': 'RandomSensor',
#                'num_sensels': self.num_sensels}
#
#    def _compute_observations(self, pose): #@UnusedVariable
#        values = np.random.rand(self.num_sensels)
#        data = {}
#        data[VehicleSensor.SENSELS] = values
#        return data
#
#    def set_world_primitives(self, primitives):
#        pass
