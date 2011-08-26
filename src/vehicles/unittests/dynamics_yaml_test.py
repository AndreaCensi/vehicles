from StringIO import StringIO
from geometry import SE3_from_R3
from vehicles.configuration import VehiclesConfig, instance_dynamics
import numpy as np
import yaml
        
def all_dynamics():
    ''' Instances all dynamics. '''
    for id_dynamics in VehiclesConfig.dynamics:
        dynamics = instance_dynamics(id_dynamics)
        yield id_dynamics, dynamics    
        
def test_state_yaml_representation():
    for id_dynamics, dynamics in all_dynamics():
        yield check_state_yaml_representation, id_dynamics, dynamics
        
def check_state_yaml_representation(id_dynamics, dynamics):
    pose = SE3_from_R3(np.array([0, 0, 0]))
    state = dynamics.pose2state(pose)
    yaml = dynamics.state_to_yaml(state)
    assert_yaml_serializable(yaml)
    
def assert_yaml_serializable(x):
    s = StringIO()
    yaml.dump(x, s)
