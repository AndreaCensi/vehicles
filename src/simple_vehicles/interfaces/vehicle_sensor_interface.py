from abc import abstractmethod


class VehicleSensor:
 
    def __init__(self, num_sensels):
        self.num_sensels = num_sensels
           
    @abstractmethod
    def set_world(self, world, updated=None):
        pass

    @abstractmethod
    def compute_observations(self, pose, dt):
        """ Computes the observations for this vehicle         
        Args: 
            pose: SE2 (world reference frame.)
            
        Returns: 
            a dictionary, containing as many data fields as desired. This data
            is eventually passed to the client as a OpenStruct.
            
            There is, however, one necessary field.
            * data['sensels'] should be the declared sensel for the sensor.
              This should be a numpy.ndarray, or a list of numbers.
              No NANs allowed. 
        """
        raise TypeError('Sensor class %s must implement compute_observations()' 
                        % type(self))
    
