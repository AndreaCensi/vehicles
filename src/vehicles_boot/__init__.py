''' Code interfacing PyVehicles with BootstrappingOlympics. '''

from vehicles import __version__, logger

try:
    import bootstrapping_olympics
    vehicles_has_boot_olympics = True
except ImportError:
    logger.error('Package BootOlympics not installed, vehicles_boot cannot '
                 'work properly.')
    vehicles_has_boot_olympics = False

if vehicles_has_boot_olympics:
    from .vehicles_simulation import *
    from .ros_visualization import *


    def get_comptests():
        # get testing configuration directory 
        from pkg_resources import resource_filename  # @UnresolvedImport
        dirname = resource_filename("vehicles_boot", "configs")
        
        from vehicles import get_vehicles_config
        get_vehicles_config().load('default')
        
        # load into bootstrapping_olympics
        from comptests import get_comptests_app
        from bootstrapping_olympics import get_boot_config
        boot_config = get_boot_config()
        boot_config.load(dirname)
        
        # Our tests are its tests with our configuration
        import bootstrapping_olympics 
        return bootstrapping_olympics.get_comptests()
