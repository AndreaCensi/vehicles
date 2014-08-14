#!/bin/bash
set -e 
set -x
where=`which vehicles_display_demo_simulations`/../../
cmake -D CMAKE_INSTALL_PREFIX=$where . 
make install

