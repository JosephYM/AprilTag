#! /bin/bash
cd `pwd`/src
catkin_init_workspace
cd ..
catkin_make
echo "source `pwd`/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd `pwd`/src
git clone https://github.com/AprilRobotics/apriltag_ros.git
git clone https://github.com/IntelRealSense/realsense-ros
git clone https://github.com/pal-robotics/ddynamic_reconfigure
cd ..

cp `pwd`/original/common_functions.cpp `pwd`/src/apriltag_ros/src/
cp `pwd`/original/settings.yaml `pwd`/src/apriltag_ros/config/
cp `pwd`/original/tags.yaml `pwd`/src/apriltag_ros/config/
cp `pwd`/original/continuous_detection_multicam.launch `pwd`/src/apriltag_ros/launch/
cp `pwd`/original/rs_l515_multi.launch `pwd`/src/realsense-ros/launch/

catkin_make