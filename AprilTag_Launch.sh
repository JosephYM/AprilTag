#! /bin/sh
echo "AprilTag enabling..."
pkill -f rs_l515_multi.launch
pkill -f continuous_detection_multicam.launch
pkill -f print_info_multicam.launch
pkill -f rviz.launch
sleep 2s

if [ "$1" = "-D" ]
then
    echo "AprilTag debugging! Please wait gnome terminal..."
    dbus-launch gnome-terminal -t "camera" -- bash -c "roslaunch realsense2_camera rs_l515_multi.launch 2>>/dev/null;exec bash"
    sleep 5s
    dbus-launch gnome-terminal -t "apriltag" -- bash -c "roslaunch apriltag_ros continuous_detection_multicam.launch;exec bash"
    sleep 5s
    dbus-launch gnome-terminal -t "info" -- bash -c "roslaunch print_info print_info_multicam.launch;exec bash"
    sleep 5s
    roslaunch  print_info rviz.launch
else
    echo "Launching cameras..."
    roslaunch realsense2_camera rs_l515_multi.launch >>/dev/null 2>&1&
    sleep 2s
    echo "Launching AprilTag..."
    roslaunch apriltag_ros continuous_detection_multicam.launch >>/dev/null 2>&1&
    sleep 2s
    echo "Launching print_info..."
    if [ "$1" = "-P" ]
    then
        roslaunch print_info print_info_multicam.launch
    else
        roslaunch print_info print_info_multicam.launch >>/dev/null 2>&1&
        sleep 2s
        echo "Launching rviz..."
        roslaunch  print_info rviz.launch >>/dev/null 2>&1
    fi
fi
rosnode kill --all
