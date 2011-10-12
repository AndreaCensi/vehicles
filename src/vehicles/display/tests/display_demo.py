from vehicles.simulation import VehicleSimulation
from vehicles.configuration.master import VehiclesConfig
from vehicles.display import display_all



def main():
    id_vehicle = 'd_SE2_rb_v-rf360'
    id_world = 'stochastic_box_10' 
    
    vehicle = VehiclesConfig.vehicles.instance(id_vehicle) #@UndefinedVariable
    world = VehiclesConfig.worlds.instance(id_world) #@UndefinedVariable
    simulation = VehicleSimulation(vehicle, world)
    
    from reprep import Report
    
    r = Report('display_demo')
    
    for i in range(10):
        sec = r.node('Section %d' % i)
        f = sec.figure()
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()
        with f.data_pylab('start', figsize=(4, 4)) as pylab:
            display_all(pylab, sim_state, grid=1, zoom=0, show_sensor_data=True)
        f.last().add_to(f)
        
    filename = 'display_demo.html'
    r.to_html(filename)
    
    
if __name__ == '__main__':
    main()
