from . import Vehicle, logger, np
from ..interfaces import World
from geometry import SE3
import time

__all__ = ['VehicleSimulation']


class VehicleSimulation():

    def __init__(self, vehicle, world, safety_margin=3):
        assert isinstance(vehicle, Vehicle)
        assert isinstance(world, World)

        self.vehicle = vehicle
        self.world = world
        self.timestamp = time.time()

        self.last_vehicle_observations = None  # used for to_yaml()
        self.last_vehicle_observations_timestamp = None  # used for to_yaml()

        self.vehicle_collided = None
        cmd_spec = self.vehicle.dynamics.get_commands_spec()
        self.last_commands = np.zeros(len(cmd_spec['format']))  # XXX

        self.episode_started = False

        self.safety_margin = safety_margin
        # 3 = must have 2*radius distant from obstacles

    def __repr__(self):
        return 'VSim(%s;%s)' % (self.vehicle, self.world)

    def info(self, s):
        logger.info(s)  # XXX

    def simulate(self, commands, dt):
        if not self.episode_started:
            # XXX better exception?
            raise ValueError('Episode not started yet.')

        commands = np.array(commands)
        updated = self.world.simulate(dt, self.vehicle)
        self.timestamp += dt

        self.vehicle.set_world_primitives(updated)
        if self.vehicle.colliding_pose(self.vehicle.get_pose()).collided:
            self.info('Collision due to dynamic world at time %.2f.' %
                       self.timestamp)
            self.vehicle_collided = True
        else:
            self.vehicle.simulate(commands, dt)
            self.vehicle_collided = self.vehicle.collision.collided
            if self.vehicle_collided:
                self.info('Collision with object at time %.2f.' %
                          (self.timestamp - self.timestamp0))

        self.last_commands = commands

    def get_timestamp(self):
        return self.timestamp
    
    def compute_observations(self):
        if not self.episode_started:
            # XXX better exception?
            raise ValueError('Episode not started yet.')

        observations = self.vehicle.compute_observations()
        self.last_vehicle_observations = observations
        self.last_vehicle_observations_timestamp = self.timestamp
        # TODO: more than one?
        return observations

    def new_episode(self):
        self.vehicle_collided = None
        self.episode_started = True
        self.timestamp = time.time()
        self.timestamp0 = self.timestamp

        max_tries = 100
        for _  in range(max_tries):
            episode = self.world.new_episode()
            # TODO: change name
            pose = episode.vehicle_state
            SE3.belongs(pose)
            primitives = self.world.get_primitives()
            check_primitives(primitives)
            self.vehicle.set_world_primitives(primitives)
            collision = self.vehicle.colliding_pose(pose,
                                    safety_margin=self.safety_margin)
            if not collision.collided:
                self.vehicle.set_pose(pose)
                return episode
            #print('Bad random: collision  %s' % str(collision))
        else:
            msg = ('Cannot find a non-colliding state after %d tries.'
                    % max_tries)
            raise Exception(msg)

    def to_yaml(self):
        ''' Returns a YAML-serializable description of the state. '''
        if not self.episode_started:
            # XXX better exception?
            raise ValueError('Episode not started yet.')

        # make sure we computed the observations at least once 
        if self.last_vehicle_observations is None:
            self.compute_observations()

        if self.timestamp != self.last_vehicle_observations_timestamp:
            logger.debug('Warning: state is at time %s, observations at %s.' %
                         (self.timestamp,
                          self.last_vehicle_observations_timestamp))

        data = {
            'version': [1, 0],
            'vehicle': self.vehicle.to_yaml(),
            'world': self.world.to_yaml(),
            'timestamp': self.timestamp,
            'observations': self.last_vehicle_observations.tolist(),
            'last_commands': self.last_commands.tolist(),
        }
        return data

    def get_vehicle(self):
        return self.vehicle

    def get_world(self):
        return self.world


def check_primitives(primitives):
    ids = [p.id_object for p in primitives]
    if len(set(ids)) != len(ids):
        msg = "Repeated IDs: %s" % ids
        msg += ' Each primitive must have a different id_object. '
        raise ValueError(msg)


