#! /bin/bash

source /opt/ros/humble/setup.bash

export CXX=clang++
export CC=clang
rm -r build 
rm -r install
rm -r log
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G "Unix Makefiles"
