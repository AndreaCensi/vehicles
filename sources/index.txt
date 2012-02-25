.. raw:: html
   :file: fork.html

.. include:: definitions.txt

.. py:currentmodule:: vehicles


Vehicles
========

|Vehicles| is a Python package useful for running 
flexible simulations of robotic vehicles.

The documentation is being written; not much is ready yet. Check back soon!

In the mean time, see `some examples of the simulations produced by this package`__.

.. __: http://www.cds.caltech.edu/~ender/pri/vehicles-videos/videos/videos-exz1sb.html

* For reference, see the `API docs`__.

.. __: static/apidocs/index.html


* `Misc demos`__.

.. __: static/demos/

..
	Simple usage example: ::
  
	      from vehicles import function
      
	      function(1,2,3)
      
	**News**

	- 2012-01-01: Version 0.2 released (:ref:`changelog <changelog>`).

	**Project status:** |Vehicles| is very well tested and documented.

	**Support**: use the GitHub issue tracker_ or email me_.

	**Documentation index**

	- :ref:`installation` 
	- :ref:`api` 
	- :ref:`api_reference`
	- :ref:`credits`


	.. _tracker: http://github.com/AndreaCensi/vehicles/issues

	.. _me: http://www.cds.caltech.edu/~andrea/


	.. _installation:

	Installation
	------------

	Install |Vehicles| using: ::

	    $ pip install Vehicles
    
	.. raw:: html
	   :file: download.html

	TYou can download this project in either zip_ or tar_ formats, or 
	download using git: ::

	    $ git clone git://github.com:AndreaCensi/vehiclesS.git

	Install using: ::

	    $ python setup.py develop
	    $ nosetests -w src         # run the extensive test suite


	.. _tar:: http://github.com/AndreaCensi/vehicles/tarball/master
	.. _zip:: http://github.com/AndreaCensi/vehicles/zipball/master


	.. include:: api.rst.inc


 

