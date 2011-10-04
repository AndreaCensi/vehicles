from bootstrapping_olympics import RobotInterface, StreamSpec, BootSpec
from vehicles import VehicleSimulation, VehiclesConfig
import numpy as np



class BOVehicleSimulation(RobotInterface, VehicleSimulation):
    
    def __init__(self, **params):
        
        self.dt = params.get('dt', 0.1) # XXX: put into constants
        
        if 'vehicle' in params:
            id_vehicle = params['vehicle']['id']
            vehicle = VehiclesConfig.vehicles.instance_spec(params['vehicle']) #@UndefinedVariable
        else:
            id_vehicle = params['id_vehicle']
            vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
            
        if 'world' in params:
            id_world = params['world']['id']
            world = VehiclesConfig.worlds.instance_spec(params['world']) #@UndefinedVariable
        else:
            id_world = params['id_world']
            world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
        
        VehicleSimulation.__init__(self, vehicle, world)
        
        RobotInterface.__init__(self)
        
        self.last_commands = np.zeros(len(self.vehicle.commands_spec)) # XXX: use spec

        cmd_spec = StreamSpec.from_yaml(self.vehicle.dynamics.get_commands_spec())
        if len(self.vehicle.sensors) == 0:
            raise Exception('Vehicle %s has no sensors defined.' % id_vehicle)
        obs_spec = None
        for attached in self.vehicle.sensors:
            sensor = attached.sensor
            ss = StreamSpec.from_yaml(sensor.observations_spec)
            obs_spec = (ss if obs_spec is None 
                        else StreamSpec.join(obs_spec, ss))
            assert obs_spec is not None
        assert obs_spec is not None
        self._boot_spec = BootSpec(obs_spec, cmd_spec, id_robot=id_vehicle) # id, desc, extra?
        
    def get_spec(self):
        return self._boot_spec
           
    def __repr__(self):
        return 'VehicleSimulation(%s,%s)' % (self.id_vehicle, self.id_world)

    def set_commands(self, commands):
        VehicleSimulation.simulate(self, commands, self.dt)

    def get_observations(self):
        observations = VehicleSimulation.compute_observations(self) 
        return self.timestamp, observations
        
    def new_episode(self):
        return VehicleSimulation.new_episode(self) 
     
