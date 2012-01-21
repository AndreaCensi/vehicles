
all:
	python setup.py install

develop:
	python setup.py develop


nose_options="-a '!simulation' --with-id"
nose_coverage="--with-coverage --cover-html --cover-html-dir coverage_information --cover-package=bootstrapping_olympics"

test:
	nosetests $(nose_options)

test-parallel:
	nosetests $(nose_options) --parallel=10

test-coverage:
	nosetests $(nose_options)   $(nose_coverage) 

docs: 
	make -C docs

print-config:
	vehicles_print_config --outdir docs/source/my_static/config/
