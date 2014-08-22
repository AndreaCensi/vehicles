from blocks import SimpleBlackBoxTN, WithQueue, check_timed_named
from bootstrapping_olympics import (
    BootOlympicsConstants, BootSpec, EpisodeDesc, ExplorableRobot, 
    RobotInterface, StreamSpec)
from contracts import contract
from vehicles import (VehicleSimulation, VehiclesConstants)
from blocks.exceptions import Exhausted

__all__ = ['BOVehicleSimulation']


class BOVehicleSimulation(RobotInterface, 
                          ExplorableRobot, 
                          VehicleSimulation):
    """ Gives a RobotInterface to the VehicleSimulation. """
    
    def __init__(self,
                 world=None,
                 vehicle=None,
                 dt=VehiclesConstants.DEFAULT_SIMULATION_DT,
                 **kwargs):
        VehicleSimulation.__init__(self, vehicle=vehicle, world=world, **kwargs)

        self.dt = dt

        cmd_spec = StreamSpec.from_yaml(
                        self.vehicle.dynamics.get_commands_spec())

        self.last_commands = cmd_spec.get_default_value()

        if len(self.vehicle.sensors) == 0:
            raise Exception('Vehicle has no sensors defined.')

        obs_spec = create_obs_spec(self.vehicle)
        self._boot_spec = BootSpec(obs_spec=obs_spec, cmd_spec=cmd_spec)
        # XXX: id, desc, extra?
        self.commands_source = BootOlympicsConstants.CMD_SOURCE_REST

        self.boot_episode_started = False

    def __repr__(self):
        return 'BOVehicleSim()' 

    # RobotInterface methods

    def get_spec(self):
        return self._boot_spec 
    
    def get_active_stream(self):
        ''' 
            Returns the stream for the interaction.
            It outputs the signals:
            
                observations
                robot_pose
                robot_state
        '''
    
        class VSimRobotStream(WithQueue, SimpleBlackBoxTN):
            def __init__(self, vsim):
                WithQueue.__init__(self)
                self.vsim = vsim
                
            def reset(self):
                WithQueue.reset(self)
                self.episode = self.vsim.new_episode()
                self.ended = False
                self._enqueue()
                assert not self.ended
                
            def _enqueue(self):
                observations = self.vsim.compute_observations()
                episode_end = True if self.vsim.vehicle_collided else False

                if episode_end:
                    self.ended = True
                    self._finished = True
                    #raise Finished()

                x = (self.vsim.timestamp, ('observations', observations))
                self.append(x)
                                     
            @contract(value='tuple(float,*)')
            def put_noblock(self, value):
                if self.ended:
                    msg = 'put() called when already ended.'
                    self.error(msg)
                    self.error('Ignoring.')
                    raise Exhausted(msg)
                
                check_timed_named(value, self)
                
                (_, (sname, x)) = value
                if not sname in ['commands']:
                    msg = 'Unexpected signal %r.' % sname
                    raise ValueError(msg)
                
                commands = x
                VehicleSimulation.simulate(self.vsim, commands, self.vsim.dt)

                self._enqueue()                
                
        return VSimRobotStream(self)

    def new_episode(self):
        self.boot_episode_started = True
        e = VehicleSimulation.new_episode(self)
        self.info("New episode started.")
        # XXX: need world id
        return EpisodeDesc(e.id_episode, 'environment') 

    def get_state(self):
        return self.to_yaml()


def create_obs_spec(vehicle):
    obs_spec = None
    for attached in vehicle.sensors:
        sensor = attached.sensor
        ss = StreamSpec.from_yaml(sensor.observations_spec)
        obs_spec = (ss if obs_spec is None
                    else StreamSpec.join(obs_spec, ss))
        assert obs_spec is not None
    assert obs_spec is not None
    return obs_spec
