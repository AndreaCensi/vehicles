from . import SimpleKinematics, contract
from geometry import (SE2, se2_from_linear_angular)
from vehicles.library.dynamics.lie_dynamics import SimpleDynamics

__all__ = ['SE2Dynamics', 'SE2Force']


class SE2Dynamics(SimpleKinematics):

    @contract(max_linear_velocity='seq[2](>=0)',
              max_angular_velocity='>=0',)
    def __init__(self, max_linear_velocity, max_angular_velocity,
                 noise_drift=None, noise_mult=None):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        spec = {
            'desc': 'Particle in SE2 controlled in velocity',
            'shape': [3],
            'format': ['C', 'C', 'C'],
            'range': [[-1, +1], [-1, +1], [-1, +1]],
            'names': ['vx', 'vy', 'angular velocity'],
            'default': [0, 0, 0],
            'extra': {'max_linear_velocity': max_linear_velocity,
                      'max_angular_velocity': max_angular_velocity,
                      'pose_space': 'SE2'}
        }
        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec,
                          noise_mult=noise_mult,
                          noise_drift=noise_drift)

    def compute_velocities(self, commands):
        linear = [commands[0] * self.max_linear_velocity[0],
                  commands[1] * self.max_linear_velocity[1]]
        angular = commands[2] * self.max_angular_velocity
        vel = se2_from_linear_angular(linear, angular)
        return vel


class SE2Force(SimpleDynamics):
    """ Body in SE2 controlled in force/torque """
    
    @contract(max_force='seq[3](>=0)',
              mass='>0',
              damping='>0')
    def __init__(self, max_force, mass, damping):
        self.max_force = max_force
        self.mass = mass
        self.damping = damping
        spec = {
            'desc': 'Particle in SE2 controlled in force/torque',
            'shape': [3],
            'format': ['C', 'C', 'C'],
            'range': [[-1, +1], [-1, +1], [-1, +1]],
            'names': ['fx', 'fy', 'torque'],
            'default': [0, 0, 0],
            'extra': {'mass': mass,
                      'damping': damping,
                      'max_force': max_force,
                      'pose_space': 'SE2'}
        }
        SimpleDynamics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec,
                          mass=mass,
                          damping=damping)

    def compute_forces(self, commands):
        linear = [commands[0] * self.max_force[0],
                  commands[1] * self.max_force[1]]
        angular = commands[2] * self.max_force[2]
        vel = se2_from_linear_angular(linear, angular)
        return vel


