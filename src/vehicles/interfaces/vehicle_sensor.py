from . import np, logger
from ..utils import check_yaml_friendly
from abc import abstractmethod, ABCMeta
from pprint import pformat
from vehicles import DO_EXTRA_CHECKS


class VehicleSensor:
    __metaclass__ = ABCMeta

    def __init__(self, observations_spec):
        check_yaml_friendly(observations_spec)
        self.observations_spec = observations_spec

    @abstractmethod
    def set_world_primitives(self, primitives):
        pass

    @abstractmethod
    def to_yaml(self, primitives):
        pass

    SENSELS = 'sensels' # TODO: move away

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
        sensels = observations[VehicleSensor.SENSELS]
        sensels = np.array(sensels)
        
        if DO_EXTRA_CHECKS:
            # todo: check spec
            #check("array[K]", sensels,
            #      desc='I expect a unidimenional array/list for sensels.')
            try:
                notfinite = not np.isfinite(sensels).all()
            except  Exception as e:
                logger.error('Error is: %s' % e) # XXX: print
                logger.error('Data is: %s' % sensels.dtype)
                logger.error((observations))
                notfinite = False
    
            if notfinite:
                msg = ('Not all are valid:\n%s\n%s' % 
                       (sensels, pformat(observations)))
                logger.error(msg)
                logger.error('pose: %s' % pose) # XXX
                # XXX: - we will solve this later.
                sensels[np.logical_not(np.isfinite(sensels))] = 0.5
                raise ValueError(msg)

                    # XXX: maybe somewhere else?
            #        if sensels.size != self.num_sensels:
            #            raise ValueError('I was expecting %d sensels, not %s.' % 
            #                             (self.num_sensels, sensels.size))

        observations[VehicleSensor.SENSELS] = sensels
        return observations

    @abstractmethod
    def _compute_observations(self, pose):
        pass

    def __repr__(self):
        return self.__class__.__name__
