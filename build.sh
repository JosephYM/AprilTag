#! /bin/bash
cd `pwd`/src
catkin_init_workspace
cd ..
# catkin_make
echo "source `pwd`/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd `pwd`/src
git clone https://github.com/AprilRobotics/apriltag_ros.git

git clone https://github.com/IntelRealSense/realsense-ros
cd realsense-ros
git reset --hard 361e0c0a9fa60634eaf3c8c67a1633fa1f09f263
cd ..

git clone https://github.com/pal-robotics/ddynamic_reconfigure 
cd ..

cp -f `pwd`/original/common_functions.cpp `pwd`/src/apriltag_ros/apriltag_ros/src/
cp -f `pwd`/original/settings.yaml `pwd`/src/apriltag_ros/apriltag_ros/config/
cp -f `pwd`/original/tags.yaml `pwd`/src/apriltag_ros/apriltag_ros/config/
cp -f `pwd`/original/continuous_detection_multicam.launch `pwd`/src/apriltag_ros/apriltag_ros/launch/
cp -f `pwd`/original/rs_l515_multi.launch `pwd`/src/realsense-ros/realsense2_camera/launch/

catkin_make
# catkin_make_isolated