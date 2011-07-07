from bootstrapping_olympics import RobotSimulationInterface
from simple_vehicles import instance_vehicle, \
    instance_world


class VehicleSimulation(RobotSimulationInterface):
    
    def __init__(self, id_vehicle, id_world):
        self.vehicle = instance_vehicle(id_vehicle)
        self.world = instance_world(id_world)
        commands_spec = self.vehicle.commands_spec
        observations_shape = self.vehicle.num_sensels
        
        RobotSimulationInterface.__init__(
                 observations_shape, commands_spec,
                 id_robot=id_vehicle,
                 id_sensors=self.vehicle.id_sensors,
                 id_actuators=self.vehicle.id_dynamics)

    def next_episode(self):
        self.id_eng
