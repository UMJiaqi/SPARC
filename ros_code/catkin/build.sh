#!/bin/sh
find . -type f -exec touch {} \;
catkin_make
