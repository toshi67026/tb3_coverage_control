cmake_minimum_required(VERSION 3.0.2)
project(tb3_coverage_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  joy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  vrpn_client_ros
  gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})
catkin_package(
  DEPENDS
    gazebo_ros
)

catkin_python_setup()
