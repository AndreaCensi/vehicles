import os
from setuptools import setup, find_packages

version = "0.1"

description = """""" 

#def read(fname):
#    return open(os.path.join(os.path.dirname(__file__), fname)).read()
#    
#long_description = read('README.rst')
    
long_description = description  

setup(name='BootstrappingOlympics',
      url='',
      
      description=description,
      long_description=long_description,
      keywords="",
      license="",
      
      classifiers=[
        'Development Status :: 4 - Beta',
        # 'Intended Audience :: Developers',
        # 'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        # 'Topic :: Software Development :: Quality Assurance',
        # 'Topic :: Software Development :: Documentation',
        # 'Topic :: Software Development :: Testing'
      ],

      version=version,
#      download_url='http://github.com/GITHUB_USER/GITHUB_PROJECT/tarball/%s' % version,
      
      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[ 'PyVehiclesDynamics', 'PyYAML'],
      tests_require=['nose'],
      entry_points={
         'console_scripts': [
           'boot_olympics_test_load_dynamics = '
                'bootstrapping_olympics.loading.dynamics:load_config_dynamics_demo',
        ]
      }          
)

