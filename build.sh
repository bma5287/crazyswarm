# Exit immediately if a command exits with a non-zero status.
set -e

ROOT=$PWD

# submodules
git submodule init
git submodule update

cd ros_ws/src/crazyflie_ros/
git submodule init
git submodule update
cd $ROOT

cd ros_ws/src/externalDependencies/libmotioncapture/
git submodule init
git submodule update
cd $ROOT


# ros
cd ros_ws
# -k: hack for dependency issues
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -k
catkin_make
cd $ROOT
