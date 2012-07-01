from .. import for_all_vehicles, get_world, for_all_skins
from ...simulation import VehicleSimulation
from vehicles_cairo import vehicles_has_cairo
import tempfile


if vehicles_has_cairo:
    from vehicles_cairo import (vehicles_cairo_display_svg,
                                vehicles_cairo_display_pdf,
                                vehicles_cairo_display_png)

    @for_all_vehicles
    def plotting(id_vehicle, vehicle):
        id_world = 'SBox2_10a'
        world = get_world(id_world)
        simulation = VehicleSimulation(vehicle, world)
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()

        plot_params = dict(grid=2,
                           zoom=vehicle.radius * 1.5,
                           width=400, height=400,
                           show_sensor_data=True)

        f = tempfile.NamedTemporaryFile(suffix='.png', delete=True)
        vehicles_cairo_display_png(filename=f.name, sim_state=sim_state,
                                   **plot_params)

        f = tempfile.NamedTemporaryFile(suffix='.pdf', delete=True)
        vehicles_cairo_display_pdf(filename=f.name, sim_state=sim_state,
                                   **plot_params)

        f = tempfile.NamedTemporaryFile(suffix='.svg', delete=True)
        vehicles_cairo_display_svg(filename=f.name, sim_state=sim_state,
                                   **plot_params)

    @for_all_skins
    def plot_skin(id_skin, skin):
        # TODO
        pass

