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

        config_dirs = [
            'vehicles.configs',
            'vehicles_boot.configs',
            'bootstrapping_olympics.configs',
        ]
        from conf_tools import GlobalConfig
        GlobalConfig.global_load_dirs(config_dirs)
        
        # load into bootstrapping_olympics
        from bootstrapping_olympics import get_boot_config
        boot_config = get_boot_config()
        
        # Our tests are its tests with our configuration
        from bootstrapping_olympics import unittests
        jobs_registrar(context, boot_config)
        
