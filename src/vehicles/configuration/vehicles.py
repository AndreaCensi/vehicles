from . import (check_valid_dynamics_config, np,
               check_valid_sensor_config, logger)
from conf_tools import (BadConfig, check_necessary, check_has_exactly_one,
    wrap_check)
from contracts import describe_type
from copy import deepcopy
from geometry import SE3_from_SE2, SE2_from_translation_angle
from pprint import pformat


def check_valid_vehicle_config(x):
    if not isinstance(x, dict):
        raise ValueError('Must be a dictionary.')
    necessary = [('id', str),
                  ('sensors', list),
                  ('radius', (float, int))]
    check_necessary(x, necessary)

    dynamics_alt = [('dynamics', dict), ('id_dynamics', str)]
    check_has_exactly_one(x, dynamics_alt)

    if 'dynamics' in x:
        wrap_check(x, 'checking "dynamics" field',
                   check_valid_dynamics_config, x['dynamics'])

    for i, s in enumerate(x['sensors']):
        wrap_check(x, 'checking #%d sensors entry' % i,
                   check_vehicle_sensor_entry, s)


def check_vehicle_sensor_entry(s):
    if not isinstance(s, dict):
        raise BadConfig(s, 'I expect this to be a dictionary, not %s' %
                        describe_type(s))
    if 'pose' in s:
        check_valid_pose_spec(s['pose'])

    alternatives = [('sensor', dict), ('id_sensor', str)]
    check_has_exactly_one(s, alternatives)
    if 'sensor' in s:
        wrap_check(s, 'checking "sensor" entry',
                   check_valid_sensor_config, s['sensor'])


def instance_vehicle_spec(entry):
    from ..simulation import Vehicle
    from . import VehiclesConfig
    from ..interfaces import VehicleSensor, Dynamics

    check_valid_vehicle_config(entry)
    try:
        master = VehiclesConfig.specs['dynamics']
        if 'id_dynamics' in entry:
            id_dynamics = entry['id_dynamics']
            dynamics = master.instance(id_dynamics)
        else:
            id_dynamics = entry['dynamics']['id']
            dynamics = master.instance_spec(entry['dynamics'])

        assert isinstance(dynamics, Dynamics)

        sensors = entry['sensors']
        radius = entry['radius']
        extra = entry.get('extra', {})
        vehicle = Vehicle(radius=radius, extra=extra)
        vehicle.add_dynamics(id_dynamics, dynamics)
        master = VehiclesConfig.specs['sensors']

        for sensor in sensors:
            if 'id_sensor' in sensor:
                id_sensor = sensor['id_sensor']
                sensor_instance = master.instance(id_sensor)
            else:
                id_sensor = sensor['sensor']['id']
                sensor_instance = master.instance_spec(sensor['sensor'])
            assert isinstance(sensor_instance, VehicleSensor)

            # TODO: document this
            x, y, theta_deg = sensor.get('pose', [0, 0, 0])
            theta = np.radians(theta_deg)
            pose = SE2_from_translation_angle([x, y], theta)
            pose = SE3_from_SE2(pose)
            joint = sensor.get('joint', 0)
            extra = sensor.get('extra', {})
            vehicle.add_sensor(id_sensor=id_sensor,
                               sensor=sensor_instance,
                               pose=pose, joint=joint,
                               extra=extra)
        return vehicle
    except:
        logger.error('Error while trying to instantiate vehicle. Entry:\n%s'
                     % (pformat(entry)))
        raise


def dereference_vehicle_spec(x):
    from . import VehiclesConfig
    ''' substitutes all references; modifies x. '''
    check_valid_vehicle_config(x)
    if 'id_dynamics' in x:
        id_dynamics = x.pop('id_dynamics')
        x['dynamics'] = deepcopy(VehiclesConfig.dynamics[id_dynamics])
    for s in x['sensors']:
        if 'id_sensor' in s:
            id_sensor = s.pop('id_sensor')
            s['sensor'] = deepcopy(VehiclesConfig.sensors[id_sensor])
    check_valid_vehicle_config(x)


def check_valid_pose_spec(s):
    pass  # XXX
