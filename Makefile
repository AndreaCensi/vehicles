package=vehicles

include pypackage.mk
	
print-config:
	vehicles_print_config --outdir docs/source/my_static/config/
