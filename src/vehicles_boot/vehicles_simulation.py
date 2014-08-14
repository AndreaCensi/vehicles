from blocks import SimpleBlackBoxTN, WithQueue, check_timed_named
from bootstrapping_olympics import (
    BootOlympicsConstants, BootSpec, EpisodeDesc, ExplorableRobot, 
    RobotInterface, StreamSpec)
from contracts import contract
from vehicles import (VehicleSimulation, VehiclesConstants, 
    get_conftools_vehicles, get_conftools_worlds)

__all__ = ['BOVehicleSimulation']


class BOVehicleSimulation(RobotInterface, 
                          ExplorableRobot, 
                          VehicleSimulation):
    """ Gives a RobotInterface to the VehicleSimulation. """
    
    def __init__(self,
                 world=None,
                 id_world=None,
                 vehicle=None,
                 id_vehicle=None,
                 dt=VehiclesConstants.DEFAULT_SIMULATION_DT,
                 **kwargs):
        self.dt = dt

        if not ((world is not None) ^ (id_world is not None)):
            raise ValueError('Specify exactly one of "world" and "id_world".')
        if not ((vehicle is not None) ^ (id_vehicle is not None)):
            raise ValueError('Specify exactly one of "vehicle" '
                             'and "id_vehicle".')

        vehicles = get_conftools_vehicles()
        worlds = get_conftools_worlds()

        # TODO: user shortcuts
        if vehicle is not None:
            id_vehicle = vehicle['id']
            vehicle_spec = vehicle
            # TODO: check well formed
            vehicle = vehicles.instance_spec(vehicle)
        else:
            vehicle_spec = vehicles[id_vehicle]
            vehicle = vehicles.instance(id_vehicle)

        if world is not None:
            id_world = world['id']
            # TODO: check well formed
            world_spec = world
            world = worlds.instance_spec(world)
        else:
            world_spec = worlds[id_world]
            world = worlds.instance(id_world)

        VehicleSimulation.__init__(self, vehicle, world, **kwargs)

        cmd_spec = StreamSpec.from_yaml(
                                self.vehicle.dynamics.get_commands_spec())

        self.last_commands = cmd_spec.get_default_value()

        if len(self.vehicle.sensors) == 0:
            raise Exception('Vehicle %r has no sensors defined.' % id_vehicle)

        obs_spec = create_obs_spec(self.vehicle)
        self._boot_spec = BootSpec(obs_spec=obs_spec, cmd_spec=cmd_spec,
                                   id_robot=id_vehicle)
        # XXX: id, desc, extra?
        self.commands_source = BootOlympicsConstants.CMD_SOURCE_REST

        self.boot_episode_started = False

        # Save for later
        self.id_world = id_world
        self.id_vehicle = id_vehicle
        self.world_spec = world_spec
        self.vehicle_spec = vehicle_spec
        
    def __repr__(self):
        return 'BOVehicleSim(%s,%s)' % (self.id_vehicle, self.id_world)

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
                    return
                
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
        return EpisodeDesc(e.id_episode, self.id_world)

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
