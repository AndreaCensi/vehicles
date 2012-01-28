from ...display import display_all
from ...simulation import VehicleSimulation
from .. import for_all_vehicles
from .. import get_world

#
#@for_all_vehicles
#def plotting(id_vehicle, vehicle):
#    id_world = 'SBox2_10a'
#    world = get_world(id_world)
#    simulation = VehicleSimulation(vehicle, world)
#    simulation.new_episode()
#    simulation.compute_observations()
#    sim_state = simulation.to_yaml()
#    import matplotlib
#    matplotlib.use('agg')
#    from matplotlib import pylab
#    display_all(pylab, sim_state, grid=1, zoom=1,
#                            show_sensor_data=True)

# TODO: add cairo tests
