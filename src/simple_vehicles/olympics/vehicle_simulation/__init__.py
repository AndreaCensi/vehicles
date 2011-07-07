from bootstrapping_olympics import RobotSimulationInterface


class VehicleSimulation(RobotSimulationInterface):
    
    def __init__(self, id_robot, id_world):
        self.vehicle = load_vehicle_from_id(id_robot)
        self.world = load_world_from_id(id_world)
        commands_spec = self.vehicle.commands_spec
        observations_shape = self.vehicle.num_sensels
        
        RobotSimulationInterface.__init__(
                 observations_shape, commands_spec,
                 id_robot=id_robot,
                 id_sensors=self.vehicle.id_sensors,
                 id_actuators=self.vehicle.id_actuators)

    def next_episode(self):
        self.id_eng
