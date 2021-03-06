from vehicles.unittests import for_all_dynamics
from StringIO import StringIO
from geometry import DifferentiableManifold, SE3_from_R3
import yaml
import numpy as np


@for_all_dynamics
def check_state_yaml_representation(id_dynamics, dynamics):  # @UnusedVariable
    pose = SE3_from_R3(np.array([0, 0, 0]))
    state = dynamics.pose2state(pose)
    yaml = dynamics.state_to_yaml(state)
    assert_yaml_serializable(yaml)


def assert_yaml_serializable(x):
    s = StringIO()
    yaml.dump(x, s)


@for_all_dynamics
def check_state_space_1a(id_dynamics, dynamics):  # @UnusedVariable
    state_space = dynamics.get_state_space()
    assert isinstance(state_space, DifferentiableManifold)


@for_all_dynamics
def check_state_space_2(id_dynamics, dynamics):  # @UnusedVariable
    pose = np.eye(4)
    state = dynamics.pose2state(pose)
    dynamics.get_state_space().friendly(state)


@for_all_dynamics
def dynamics_to_str(id_dynamics, dynamics):  # @UnusedVariable
    dynamics.__str__()


