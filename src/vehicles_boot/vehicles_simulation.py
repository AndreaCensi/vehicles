from bootstrapping_olympics import (EpisodeDesc, RobotInterface, StreamSpec,
    BootSpec, RobotObservations, BootOlympicsConstants)
from vehicles import VehicleSimulation, VehiclesConfig, VehiclesConstants

class BOVehicleSimulation(RobotInterface, VehicleSimulation):
    
    def __init__(self,
                 world=None,
                 id_world=None,
                 vehicle=None,
                 id_vehicle=None,
                 dt=VehiclesConstants.DEFAULT_SIMULATION_DT):
        self.dt = dt
        
        if not ((world is not None) ^ (id_world is not None)):
            raise ValueError('Specify exactly one of "world" and "id_world".')
        if not ((vehicle is not None) ^ (id_vehicle is not None)):
            raise ValueError('Specify exactly one of "vehicle" and "id_vehicle".')
            
        if vehicle is not None:
            id_vehicle = vehicle['id']
            # TODO: check well formed
            vehicle = VehiclesConfig.vehicles.instance_spec(vehicle) #@UndefinedVariable
        else:
            vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
            
        if world is not None:
            id_world = world['id']
            # TODO: check well formed
            world = VehiclesConfig.worlds.instance_spec(world) #@UndefinedVariable
        else:
            world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
        
        self.id_world = id_world
        self.id_vehicle = id_vehicle
        
        VehicleSimulation.__init__(self, vehicle, world)
        
        cmd_spec = StreamSpec.from_yaml(self.vehicle.dynamics.get_commands_spec())
        
        self.last_commands = cmd_spec.get_default_value()

        if len(self.vehicle.sensors) == 0:
            raise Exception('Vehicle %r has no sensors defined.' % id_vehicle)

        obs_spec = create_obs_spec(self.vehicle)
        self._boot_spec = BootSpec(obs_spec=obs_spec, cmd_spec=cmd_spec,
                                   id_robot=id_vehicle) 
        # XXX: id, desc, extra?
        self.commands_source = BootOlympicsConstants.CMD_SOURCE_REST

    def __repr__(self):
        return 'BOVehicleSim(%s,%s)' % (self.id_vehicle, self.id_world)
    
    # RobotInterface methods
    
    def get_spec(self):
        return self._boot_spec           

    def set_commands(self, commands, commands_source):
        self.commands_source = commands_source
        VehicleSimulation.simulate(self, commands, self.dt)

    def get_observations(self):
        observations = VehicleSimulation.compute_observations(self)
        return RobotObservations(timestamp=self.timestamp,
                         observations=observations,
                         commands=self.last_commands,
                         commands_source=self.commands_source,
                         robot_pose=self.vehicle.get_pose(),
                         episode_end=True if self.vehicle_collided else False)

    def new_episode(self):
        e = VehicleSimulation.new_episode(self)
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
