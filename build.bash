ROS_DISTRO=lyrical
source /opt/ros/$ROS_DISTRO/setup.bash

echo "Removing build trash ..."
rm -rf build install log

echo "Building topics broadcaster package ..."
colcon build --packages-select topics_broadcaster_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Installing package ..."
source ./install/local_setup.bash

echo "Finished"
