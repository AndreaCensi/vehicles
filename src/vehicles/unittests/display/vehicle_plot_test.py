from vehicles.display.vehicle_display_utils import display_all
from vehicles.simulation.simulation import VehicleSimulation
from vehicles.unittests.generation import for_all_vehicles
from vehicles.unittests.instantiation import get_world


@for_all_vehicles
def plotting(id_vehicle, vehicle):
    id_world = 'SBox2_10a'
    world = get_world(id_world)
    simulation = VehicleSimulation(vehicle, world)
    simulation.new_episode()
    simulation.compute_observations()
    sim_state = simulation.to_yaml()
    import matplotlib
    matplotlib.use('agg')
    from matplotlib import pylab
    display_all(pylab, sim_state, grid=1, zoom=1,
                            show_sensor_data=True)
