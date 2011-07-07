from bootstrapping_olympics import RobotSimulationInterface
from simple_vehicles import instance_vehicle, \
    instance_world


class VehicleSimulation(RobotSimulationInterface):
    
    def __init__(self, id_vehicle, id_world):
        self.id_vehicle = id_vehicle
        self.id_world = id_world
        self.vehicle = instance_vehicle(id_vehicle)
        self.world = instance_world(id_world)
        commands_spec = self.vehicle.commands_spec
        observations_shape = self.vehicle.num_sensels
        
        RobotSimulationInterface.__init__(self,
                 observations_shape, commands_spec,
                 id_robot=id_vehicle,
                 id_sensors=self.vehicle.id_sensors,
                 id_actuators=self.vehicle.id_dynamics)
    
    def __repr__(self):
        return 'VehicleSimulation(%s,%s)' % (self.id_vehicle, self.id_world)

    def simulate(self, commands, dt):
        self.vehicle.simulate(commands, dt)
    
    def compute_observations(self):
        return self.vehicle.compute_observations()
    
    def new_episode(self):
        episode = self.world.new_episode()
        self.id_episode = episode.id_episode
        self.vehicle.set_state(episode.vehicle_state)
        self.vehicle.set_world(self.world)
        return episode
