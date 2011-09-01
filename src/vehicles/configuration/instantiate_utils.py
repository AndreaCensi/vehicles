from contracts import contract, check
from . import logger
from contracts.interface import describe_value

@contract(code_spec='seq[2]')
def instantiate_spec(code_spec):
    ''' code_spec must be a sequence  [string, dictionary], giving
        the python function (or class) to instantiate, along
        with its parameters. '''
    function_name = code_spec[0]
    parameters = code_spec[1]
    check('str', function_name)
    check('dict', parameters)
    return instantiate(function_name, parameters)


@contract(code_spec='seq[2]')
def instantiate_spec_and_check(code_spec, expected_type):
    ''' Simple wrapper around instantiate_spec that 
        checks the type. '''
    x = instantiate_spec(code_spec)
    if not isinstance(x, expected_type):
        msg = 'Error in instantiating code spec:\n\t%s\n' % code_spec
        msg += 'I expected a %s, got %s' % (expected_type, describe_value(x)) 
        raise Exception(msg)
    return x  
    
# 

def instantiate(function_name, parameters):
    function = import_name(function_name)
    try:
        return function(**parameters)
    except TypeError as e:
        msg = ('Exception while instantiating [%r, %s]:\n\t%s' % 
               (function_name, parameters, e))
        logger.error(msg)
        raise
    



@contract(name='str')
def import_name(name):
    ''' 
        Loads the python object with the given name. 
    
        Note that "name" might be "module.module.name" as well.
    '''
    try:     
        return __import__(name, fromlist=['dummy'])
    except ImportError as e:
        # split in (module, name) if we can
        if '.' in name:
            tokens = name.split('.')
            field = tokens[-1]
            module_name = ".".join(tokens[:-1])
            
            try: 
                module = __import__(module_name, fromlist=['dummy'])
            except ImportError as e:
                msg = 'Cannot load %r (tried also with %r): %s.' % (name, module_name, e)
                raise Exception(msg)
            
            if not field in module.__dict__:
                msg = 'No field %r found in module %r.' % (field, module)
                raise Exception(msg) 
            
            return module.__dict__[field]
        else:
            msg = 'Cannot import name %r, and cannot split: %s' % (name, e)
            raise Exception(msg)
    
