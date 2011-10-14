from vehicles import __version__

try:
	import bootstrapping_olympics
	boot_installed = True
except ImportError:
	boot_installed = False

if boot_installed:
	from .vehicles_simulation import *
	from .ros_visualization import *
else:
	import warnings
	warnings.warn('Package BootOlympics not installed.')
