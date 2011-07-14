import os
import fnmatch
import yaml #@UnresolvedImport
from contracts import check
from .. import logger
from contracts import contract
from pprint import pformat

def locate_files(directory, pattern):
    for root, dirs, files in os.walk(directory): #@UnusedVariable
        for f in files: 
            if fnmatch.fnmatch(f, pattern):
                yield os.path.join(root, f)


@contract(directory='str',
          pattern='str')
def load_configuration_entries(directory, pattern, check_entry):
    ''' 
        Loads all .dynamics.yaml files recursively in the directory. 
        It is assumed that each file contains a list of dictionaries.
        Each dictionary MUST have the field 'id' and it must be unique.
        Moreover, it MUST have other fields as specified in required_fields.
        Moreover, it CANNOT have other fields other than 'id', 
        required_fields, optional_fields. 
    '''
    
    def enumerate_entries():
        for filename in locate_files(directory, pattern):
            filename = os.path.realpath(filename)
            with open(filename) as f:
                parsed = yaml.load(f)
                if parsed is None:
                    raise Exception('Empty file %r.' % filename) 
                check('list(dict)', parsed)
                if not parsed:
                    raise Exception('Empty file %r.' % filename) 
                for num_entry, entry in enumerate(parsed): 
                    yield (filename, num_entry), entry 
#    
#    allowed_fields = set(['id'] + required_fields + optional_fields)
#    required_fields = set(['id'] + required_fields)
#    def check_entry(x):
#        for field in required_fields:
#            if not field in x:
#                raise Exception('Entry does not have field %r.' % (field))
#        if check_not_allowed:
#            for found in x:
#                if not found in allowed_fields:
#                    msg = 'Field %r not allowed in entry.' % (found)
#                    raise Exception(msg)        
    name2where = {}
    all_entries = {}
    for where, x in enumerate_entries():
        try: 
            check_entry(x)
            # Warn about this?
            name = x['id']
            if name in all_entries and name2where[name] != where:
                msg = ('While loading %r from:\n %s (entry #%d),\n'
                       'already know from:\n %s (entry #%d)' % 
                                (name, where[0], where[1],
                                  name2where[name][0], name2where[name][1]))
                raise Exception(msg)
            name2where[name] = where
            all_entries[name] = x
        except Exception as e:
            msg = ('Error while loading entry from file %r #%d.\n%s\nError: %s' % 
                   (where[0], where[1], pformat(x), e))
            logger.error(msg) # XXX
            raise    
    return all_entries
    
