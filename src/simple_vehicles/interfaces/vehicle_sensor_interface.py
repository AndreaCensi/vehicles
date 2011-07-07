from abc import abstractmethod, ABCMeta
import numpy as np

class VehicleSensor:
    __metaclass__ = ABCMeta
    
    def __init__(self, num_sensels):
        self.num_sensels = num_sensels
           
    @abstractmethod
    def set_world(self, world, updated=None):
        pass

    SENSELS = 'sensels'
    
    def compute_observations(self, pose):
        """ Computes the observations for this vehicle         
        Args: 
            pose: SE2 (world reference frame.)
            
        Returns: 
            a dictionary, containing as many data fields as desired. This data
            is eventually passed to the client as a OpenStruct.
            
            There is, however, one necessary field.
            * data[SENSELS] should be the declared sensel for the sensor.
              This should be a numpy.ndarray, or a list of numbers.
              No NANs allowed. 
        """
        observations = self._compute_observations(pose)
        if not isinstance(observations, dict):
            raise ValueError('This should return a dict()')
        if not VehicleSensor.SENSELS in observations:
            raise ValueError('No field %r in %r' % 
                            (VehicleSensor.SENSELS, observations.keys()))
        sensels = observations[VehicleSensor.SENSELS ]
        sensels = np.array(sensels)
        if not np.isfinite(sensels).all():
            msg = 'Not all are valid:\n%s' % sensels
            raise ValueError(msg)
        
        if sensels.size != self.num_sensels:
            raise ValueError('I was expecting %d sensels, not %s.' % 
                             (self.num_sensels, sensels.size))
        observations[VehicleSensor.SENSELS] = sensels
        return observations

    @abstractmethod
    def _compute_observations(self, pose):
        pass
    
    def __repr__(self):
        return self.__class__.__name__
