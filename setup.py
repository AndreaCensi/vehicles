import os
from setuptools import setup, find_packages

version = "2.1"

description = """"""  # TODO

long_description = description    # TODO

setup(name='PyVehicles',
      url='',
      description=description,
      long_description=long_description,
      package_data={'':['*.*']},
      keywords="",
      license="",

      classifiers=[
        'Development Status :: 4 - Beta',
      ],

      version=version,

      download_url=
        'http://github.com/AndreaCensi/vehicles/tarball/%s' % version,

      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[
        'ConfTools>=1.0,<2',
        'PyContracts>=1.2,<2',
        'PyYAML',
        'python-cjson',
        'PyGeometry'
      ],
      tests_require=['nose>=1.1.2,<2'],
      extras_require={ # TODO: reprep?
        'procgraph':  ["procgraph>=1.0,<2"],
        'boot':  ["BootOlympics>=1.0,<2"],
      },
      entry_points={
         'console_scripts': [
           'vehicles_print_config = '
                'vehicles.programs.print_config:main',
           'vehicles_display_demo_simulations = '
                'vehicles.programs.display_demos.simulations:main',
           'vehicles_display_demo_skins = '
                'vehicles.programs.display_demos.skins:main',
           'vehicles_display_demo_vehicles = '
                'vehicles.programs.display_demos.vehicles:main',
           'vehicles_inspect_textures = '
                'vehicles.programs.inspect_textures:main',
            'vehicles_create_olympics_configs = '
                'vehicles_boot.create_olympics_configs:main',
           'vehicles_fps = '
                'vehicles.unittests.fps_test:main',
        ]
      }
)

