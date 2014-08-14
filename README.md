

Installation
------------

Install the python package:

    python setup.py develop

Install the raytracer executable:

    sudo apt-get install cmake
    cd src/raytracer
    ./install.sh


Installation notes for py2cairo + EPD 7.3 + OS X
-------------------------------------------------

Download:

    wget http://cairographics.org/releases/py2cairo-1.10.0.tar.bz2
    tar xzvf pycairo-1.10.0

Standard installation sequence

    ./waf configure --prefix $PWD/../..

    PREFIX=/data/work/scm/environments/env_fault/deploy/
    ./waf configure --prefix $PREFIX --libdir $PREFIX/lib 
    ./waf build
    ./waf install
    python -c "import cairo"



Problem on OS X:

    python -c "import cairo"      
    Fatal Python error: PyThreadState_Get: no current thread[1]   
    79043 abort      python -c "import cairo"

Run:
    
    ./waf clean
    ./waf build -vv

The last compilation is wrong, because it links python in /opt/local/lib:

    /usr/bin/gcc src/cairomodule.c.1.o src/context.c.1.o src/font.c.1.o src/path.c.1.o src/pattern.c.1.o src/matrix.c.1.o src/surface.c.1.o -o /data/work/scm/env_bv/src-third/py2cairo-1.10.0/build_directory/src/_cairo.so -L/opt/local/lib -lcairo -lpython2.7 -L/Library/Frameworks/EPD64.framework/Versions/7.3//lib -dynamiclib -arch x86_64 -arch x86_64 -arch x86_64 -arch x86_64'

We need to add -L/Library/Frameworks/EPD64.framework/Versions/7.3//lib  before '-lpython2.7'

    /usr/bin/gcc src/cairomodule.c.1.o src/context.c.1.o src/font.c.1.o src/path.c.1.o src/pattern.c.1.o src/matrix.c.1.o src/surface.c.1.o -o /data/work/scm/env_bv/src-third/py2cairo-1.10.0/build_directory/src/_cairo.so -L/Library/Frameworks/EPD64.framework/Versions/7.3//lib -L/opt/local/lib -lcairo  -lpython2.7 -dynamiclib -arch x86_64 -arch x86_64 -arch x86_64 -arch x86_64

Now install:
   
    ./waf install


