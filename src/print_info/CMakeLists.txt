cmake_minimum_required(VERSION 3.0.2)
project(print_info)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  apriltag_ros
  visualization_msgs
  cv_bridge
  image_transport
  tf2 
  tf2_ros
)

find_package(OpenCV 4.0 REQUIRED)
message(STATUS "${OpenCV_LIBRARIES}")

find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs cv_bridge image_transport tf2 tf2_ros
)

include_directories(
  include
  include/print_info
  ${catkin_INCLUDE_DIRS}
)

add_executable(print_info_node_intel src/print_info_node.cpp)
add_executable(polling_display_node_intel src/polling_display_node.cpp)
add_executable(show_theory_pose_node_intel src/show_theory_pose_node.cpp)

target_link_libraries(print_info_node_intel
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(polling_display_node_intel
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(show_theory_pose_node_intel
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)