from ..loading import load_configuration, Configuration
from simple_vehicles.loading.instance_all import instance_all


def test_configuration():
    ''' Simply load the configuration. '''    
    load_configuration()
    

def test_intantiation():
    if not Configuration.loaded:
        load_configuration()
    instance_all()
