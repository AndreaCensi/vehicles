from geometry import SE2_from_translation_angle
import numpy as np
from vehicles.worlds.boxes import Box
from vehicles.sensors.raytracer.textured_raytracer import Raytracer, \
    TexturedRaytracer


def test_raytracer_box():
    world = Box(10, 10)
    sensor = Raytracer(directions=np.linspace(0, np.pi * 2, 181))
    sensor.set_world_primitives(world.get_primitives())
    pose = SE2_from_translation_angle([0, 0], 0)
    observations = sensor.raytracing(pose)
    readings = np.array(observations['readings'])
    #coords = np.array(observations['curvilinear_coordinate'])

    #    print observations

    assert np.array(observations['valid']).all()
    assert (readings <= 10 * np.sqrt(2) + 0.0001).all()
    assert (readings >= 10).all()

    hit, _ = sensor.query_circle([0, 0], 1)
    assert not hit
    hit, _ = sensor.query_circle([0, 0], 11)
    assert hit


def test_textured_raytracer_box():
    world = Box(10, 10)
    sensor = TexturedRaytracer(directions=np.linspace(-np.pi / 2,
                                                      + np.pi / 2, 181))
    sensor.set_world_primitives(world.get_primitives())
    pose = SE2_from_translation_angle([0, 0], 0)
    sensor.raytracing(pose)
    # TODO: incomplete
