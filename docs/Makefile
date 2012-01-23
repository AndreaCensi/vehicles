# v. 1.5
demos_root=source/my_static/demos
github_repo=git@github.com:AndreaCensi/vehicles.git

# also change this in epydoc.cfg
webdir=website
webgit=git --git-dir $(webdir)/.git --work-tree=$(webdir)/ 

all: compile-website 
	
compile-website: website epydoc
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -E -n -a -b html source website

compile:
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -n -a -b html source website
	
website: distclean
	# Check out the website
	git clone $(github_repo) $@
	$(webgit) checkout origin/gh-pages -b gh-pages
	$(webgit) branch -D master

upload:
	$(webgit) commit -m "updated website" -a 
	$(webgit) push

distclean:
	rm -rf $(webdir)/
	
epydoc:
	epydoc --config epydoc.cfg --introspect-only -v --exclude-introspect=vehicles.unittests --debug

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
	
