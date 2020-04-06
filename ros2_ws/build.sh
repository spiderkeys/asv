# Install conan deps
set -e

source /opt/ros/dashing/setup.bash

mkdir -p src/.build

conan install --build missing -if src/.build .

colcon build --event-handlers 
# --cmake-args 
#-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON 
#-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.0