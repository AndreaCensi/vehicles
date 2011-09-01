from StringIO import StringIO
from geometry import DifferentiableManifold, SE3_from_R3
from vehicles import VehiclesConfig, instance_dynamics
import numpy as np
import yaml
from types import NoneType
        
def all_dynamics():
    ''' Instances all dynamics. '''
    for id_dynamics in VehiclesConfig.dynamics:
        dynamics = instance_dynamics(id_dynamics)
        yield id_dynamics, dynamics    
        
def test_state_yaml_representation():
    for id_dynamics, dynamics in all_dynamics():
        yield check_state_yaml_representation, id_dynamics, dynamics
        yield check_state_space_1a, id_dynamics, dynamics
        yield check_state_space_1b, id_dynamics, dynamics
        yield check_state_space_1c, id_dynamics, dynamics
        yield check_state_space_2, id_dynamics, dynamics
        
def check_state_yaml_representation(id_dynamics, dynamics):
    pose = SE3_from_R3(np.array([0, 0, 0]))
    state = dynamics.pose2state(pose)
    yaml = dynamics.state_to_yaml(state)
    assert_yaml_serializable(yaml)
    
def assert_yaml_serializable(x):
    s = StringIO()
    yaml.dump(x, s)

def check_state_space_1a(id_dynamics, dynamics):
    state_space = dynamics.get_state_space()
    assert isinstance(state_space, DifferentiableManifold)

def check_state_space_1b(id_dynamics, dynamics):
    pose_space = dynamics.get_pose_space()
    assert isinstance(pose_space, DifferentiableManifold)

def check_state_space_1c(id_dynamics, dynamics):
    shape_space = dynamics.get_shape_space()
    assert isinstance(shape_space, (DifferentiableManifold, NoneType))

def check_state_space_2(id_dynamics, dynamics):
    pose = np.eye(4)
    state = dynamics.pose2state(pose)
    dynamics.get_state_space().friendly(state)
    
    
def test_dynamics_to_str():
    for id_dynamics, dynamics in all_dynamics(): #@UnusedVariable
        dynamics.__str__()
        

