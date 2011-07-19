from contracts.interface import describe_type
from contracts import contract
from pprint import pformat

class BadConfig(Exception):
    def __init__(self, structure, complaint):
        self.structure = structure
        self.complaint = str(complaint)
        self.stack = []
    
    def context(self, structure):
        self.stack.append(structure)
        
    def __str__(self):
        s = '%s\n' % self.complaint
        s += 'Bad configuration snippet:\n%s\n\n' % pformat(self.structure)
        for x in self.stack:
            s += '%s\n' % x 
        return s
    
def wrap_check(x, what, function, *arg, **args):
    try:
        function(*arg, **args)
    except BadConfig as e:
        e.context('- While %s for structure:\n%s' % (what, pformat(x))) 
        raise e
    
@contract(necessary='list(tuple(str,*))')
def check_necessary(x, necessary):
    for f, types in necessary:
        if not f in x:
            raise BadConfig(x, 'Field %r missing.' % f)
        if not isinstance(x[f], types):
            msg = ('Field %r must be one of %s, instead of %s.' % 
                   (f, types, describe_type(x[f])))
            raise BadConfig(x, msg)

def check_has_exactly_one(x, alternatives):
    found = []
    for f, types in alternatives:
        if f in x:
            found.append(f)
            if not isinstance(x[f], types):
                msg = ('Field %r must be one of %s, instead of %s.' % 
                       (f, types, describe_type(x[f])))
                raise BadConfig(x, msg)

    if not found:
        msg = ('The entry must have at least one in %r.' % 
                         [ft[0] for ft in alternatives])
        raise BadConfig(x, msg)
    
    if len(found) > 1:
        msg = ('The entry cannot have all of these together: %r' % found)
        raise BadConfig(x, msg) 

def check_valid_pose_spec(s):
    pass
    # XXX
    
def check_generic_code_desc(x, what):
    """ Checks that it has fields id,desc,code and code is a code spec. """
    if not isinstance(x, dict):
        raise BadConfig(x, 'A valid %s config must be a dictionary.' % what)
    necessary = [ 
                 ('id', str),
                  ('desc', str),
                  ('code', list),
                  ]
    check_necessary(x, necessary)
    wrap_check(x, 'checking "code" entry', check_valid_code_spec, x['code'])

    
def check_valid_code_spec(x):
    if not isinstance(x, list):
        raise BadConfig(x, 'A code spec must be a list.')
    
    if len(x) != 2:
        raise BadConfig(x, 'A code spec must be a list of exactly two elements.')
    
    name = x[0]
    params = x[1]
    if not isinstance(name, str):
        raise BadConfig(x, 'The code must be given as a string.')
    if not isinstance(params, dict):
        raise BadConfig(x, 'The code params be given as a dictionary.')

def check_valid_sensor_config(x):
    return check_generic_code_desc(x, 'sensor')

def check_valid_world_config(x):
    return check_generic_code_desc(x, 'world')

def check_valid_dynamics_config(x):
    return check_generic_code_desc(x, 'dynamics')


def check_valid_vehicle_config(x):
    if not isinstance(x, dict):
        raise ValueError('Must be a dictionary.')
    necessary = [ ('id', str),
                  ('sensors', list),
                  ('radius', (float, int)),
                  ]
    check_necessary(x, necessary)
    
    dynamics_alt = [ ('dynamics', dict), ('id_dynamics', str)]
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
    if not 'pose' in s:
        raise ValueError('Missing field "pose".')
    check_valid_pose_spec(s['pose'])
    
    alternatives = [ ('sensor', dict), ('id_sensor', str)]
    check_has_exactly_one(s, alternatives)
    if 'sensor' in s:
        wrap_check(s, 'checking "sensor" entry',
                   check_valid_sensor_config, s['sensor'])
                   
def check_valid_simulation_config(x):
    vehicles_alt = [ ('vehicle', dict), ('id_vehicle', str)]
    check_has_exactly_one(x, vehicles_alt)

    if 'vehicle' in x:
        wrap_check(x, 'checking "vehicle" field',
                   check_valid_vehicle_config, x['vehicle'])

    world_alt = [ ('world', dict), ('id_world', str)]
    check_has_exactly_one(x, world_alt)

    if 'world' in x:
        wrap_check(x, 'checking "world" field',
                   check_valid_world_config, x['world'])
    
