import numpy as np

BLACK = [0, 0, 0]
WHITE = [1, 1, 1]
YELLOW = [1, 1, 0]

__all__ = ['CairoConstants', 'BLACK', 'WHITE', 'YELLOW']

class CairoConstants(object):
    obstacle_border_width = 0.01
    obstacle_border_color = [0, 0, 0]
    obstacle_fill_color = [0.7, 0.7, 0.8]

    omni_wheel_style = dict(border_color=BLACK,
                            border_width=0.01,
                            fill_color=[.3, .3, .3]
#                            fill_color=[.3, .3, 1]
                            )

    robot_wheel_style = dict(border_color=BLACK,
                             border_width=0.04,
                            fill_color=[.3, .3, .3])

    grid_color = [0.8, 0.8, 0.8]
    grid_width = 0.005

    robot_body_style = dict(border_width=0.01,
                            border_color=BLACK,
                            fill_color=[0.8, 0.8, 0.8])

    robot_border_width = 0.01
    robot_border_color = BLACK
    robot_fill_color = [0.8, 0.8, 0.8]

    robot_wheel_fill_color = [.2, .2, .2]
    robot_wheel_border_width = 0.04
    robot_wheel_border_color = BLACK

    texture_resolution = 0.05  # meters
    texture_width = 0.2  # meters

    sensor_data_plot_compact = True

    # red bg
    # laser_compact_bg = [1, 0.6, 0.6, 0.3]
    # laser_compact_fg = [1, 0, 0, 0.8]
    # yellow?
#     laser_compact_bg = [1, 1, 0.6, 0.3]
#     laser_compact_fg = [1, 1, 0, 0.8]
    # yellowish
    laser_compact_bg = [1, 0.85, 0.6, 0.3]
    laser_compact_fg = [1, 0.85, 0, 0.8]
    laser_compact_r = 0.4
    laser_compact_r_width = 0.25

    photoreceptors_compact_r = 0.4
    photoreceptors_compact_r_width = 0.15
    
    field_sampler_color0 = [178 / 255.0, 215 / 255.0, 140 / 255.0, 0.3]
    field_sampler_color1 = [188 / 255.0, 225 / 255.0, 150 / 255.0, 1]

    plot_ranges_rho_min = 0.5

    delta_single_ray = np.deg2rad(30)

