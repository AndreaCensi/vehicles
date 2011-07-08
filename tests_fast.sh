#!/bin/bash
set -e
set -x
nosetests -a '!simulation' -w src $*