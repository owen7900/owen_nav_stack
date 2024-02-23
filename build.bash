#! /bin/bash

source /opt/ros/humble/setup.bash

export CXX=clang++
export CC=clang
# rm -r build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G "Unix Makefiles"
