github_repo=git@github.com:AndreaCensi/vehicles.git

include docs.mk

generate-custom: demos summary

summary:
	./list_stuff.py -i vehicles.World vehicles.library.worlds > source/api/worlds.rst.inc
	./list_stuff.py -i vehicles.VehicleSensor vehicles.library.sensors > source/api/sensors.rst.inc
	./list_stuff.py -i vehicles.Dynamics vehicles.library.dynamics > source/api/dynamics.rst.inc
	./list_stuff.py -i vehicles.Primitive vehicles > source/api/primitives.rst.inc
	./list_stuff.py -i vehicles.VehicleSkin vehicles_cairo.skins > source/api/skins.rst.inc
	./list_stuff.py  vehicles.library.noises > source/api/noises.rst.inc
	./list_stuff.py -i vehicles.Texture vehicles.library.textures > source/api/textures.rst.inc


demos_root=source/my_static/demos

demos: demo-skins demo-vehicles demo-sims demo-textures
	mkdir -p $(demos_root)
	vehicles_print_config --outdir $(demos_root)/config

demo-sims:
	vehicles_display_demo_simulations --outdir $(demos_root)/simulations

demo-skins:
	vehicles_display_demo_skins       --outdir $(demos_root)/skins

demo-vehicles:
	vehicles_display_demo_vehicles    -d default --outdir $(demos_root)/vehicles

demo-textures:
	vehicles_inspect_textures         --outdir $(demos_root)/textures
	
