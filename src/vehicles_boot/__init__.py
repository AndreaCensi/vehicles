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
    #from .ros_visualization import *

    def jobs_comptests(context):
        from comptests import jobs_registrar

        from vehicles import get_vehicles_config
        get_vehicles_config().load('default')

        # get testing configuration directory 
        from pkg_resources import resource_filename  # @UnresolvedImport
        dirname = resource_filename("vehicles_boot", "configs")
        
        from conf_tools import GlobalConfig
        GlobalConfig.global_load_dir(dirname)
        
        # load into bootstrapping_olympics
        from bootstrapping_olympics import get_boot_config
        boot_config = get_boot_config()
#         boot_config.load(dirname)
        
        # Our tests are its tests with our configuration
        from bootstrapping_olympics import unittests
        j1 = jobs_registrar(context, boot_config)
        return j1
