from reprep import MIME_PDF, MIME_PNG, MIME_SVG, Report
from vehicles import VehicleSimulation, get_conftools_worlds
from vehicles.unittests.generation import (for_all_skins, 
    for_all_vehicles_context, for_all_worlds_context)
from vehicles_cairo import vehicles_has_cairo


if vehicles_has_cairo:
    from vehicles_cairo import (vehicles_cairo_display_svg,
                                vehicles_cairo_display_pdf,
                                vehicles_cairo_display_png)

    @for_all_worlds_context
    def plotting_world(context, id_world, world):  # @UnusedVariable
#         XXX: need one with field sampler
#         vehicle = 'd_SE2_dd_v-fs_05_12x12'
        vehicle = 'example_custom_vehicle'
        simulation = VehicleSimulation(vehicle, world)
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()

        plot_params = dict(grid=2,
                           zoom=0,
                           width=400, height=400,
                           show_sensor_data=True)

        c = context
        c.add_report(c.comp_config(report_plot1, sim_state, plot_params), 
                     'plot_world_png')
        c.add_report(c.comp_config(report_plot2, sim_state, plot_params), 
                     'plot_world_pdf')
        c.add_report(c.comp_config(report_plot3, sim_state, plot_params), 
                     'plot_world_svg')
        

    @for_all_vehicles_context
    def plotting_vehicle(context, id_vehicle, vehicle):  # @UnusedVariable
        id_world = 'SBox2_10a'
        world = get_conftools_worlds().instance(id_world)
        simulation = VehicleSimulation(vehicle, world)
        simulation.new_episode()
        simulation.compute_observations()
        sim_state = simulation.to_yaml()

        plot_params = dict(grid=2,
                           zoom=vehicle.radius * 1.5,
                           width=400, height=400,
                           show_sensor_data=True)

        c = context
        c.add_report(c.comp_config(report_plot1, sim_state, plot_params), 
                     'plot_vehicle_png')
        c.add_report(c.comp_config(report_plot2, sim_state, plot_params), 
                     'plot_vehicle_pdf')
        c.add_report(c.comp_config(report_plot3, sim_state, plot_params), 
                     'plot_vehicle_svg')
        
    def report_plot1(sim_state, plot_params):
        r = Report()
        f = r.figure()
        with f.data_file('plot1', MIME_PNG) as filename:
            vehicles_cairo_display_png(filename, sim_state=sim_state,
                                   **plot_params)
        #r.text('state', pformat(sim_state))
        return r
    
    def report_plot2(sim_state, plot_params):
        r = Report()
        f = r.figure()
        with f.data_file('plot2',  MIME_PDF) as filename:
            vehicles_cairo_display_pdf(filename, sim_state=sim_state,
                                   **plot_params)
        #r.text('state', pformat(sim_state))
        return r

    def report_plot3(sim_state, plot_params):
        r = Report()
        f = r.figure()    
        with f.data_file('plot3',  MIME_SVG) as filename:
            vehicles_cairo_display_svg(filename, sim_state=sim_state,
                                   **plot_params)
        #r.text('state', pformat(sim_state))
        return r

    @for_all_skins
    def plot_skin(id_skin, skin):
        # TODO
        pass

