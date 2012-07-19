from bootstrapping_olympics import (EpisodeDesc, RobotInterface, StreamSpec,
    BootSpec, RobotObservations, BootOlympicsConstants)
from vehicles import VehicleSimulation, VehiclesConfig, VehiclesConstants

__all__ = ['BOVehicleSimulation']


class BOVehicleSimulation(RobotInterface, VehicleSimulation):
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

        vehicles = VehiclesConfig.specs['vehicles']
        worlds = VehiclesConfig.specs['worlds']

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

    def set_commands(self, commands, commands_source):
        if not self.boot_episode_started:
            raise Exception('set_commands() called before new_episode().')

        self.commands_source = commands_source
        VehicleSimulation.simulate(self, commands, self.dt)

    def get_observations(self):
        if not self.boot_episode_started:
            raise Exception('get_observations() called before new_episode().')

        observations = VehicleSimulation.compute_observations(self)
        episode_end = True if self.vehicle_collided else False
        if episode_end:
            self.info("Ending boot episode due to collision.")
            self.boot_episode_started = False

        return RobotObservations(timestamp=self.timestamp,
                         observations=observations,
                         commands=self.last_commands,
                         commands_source=self.commands_source,
                         robot_pose=self.vehicle.get_pose(),
                         episode_end=episode_end)

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
