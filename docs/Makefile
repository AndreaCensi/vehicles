github_repo=git@github.com:AndreaCensi/vehicles.git

include docs.mk
	
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
	
