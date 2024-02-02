cmake_minimum_required(VERSION 3.0.2)
project(tiago_obst_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tiago_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tiago_obst_avoidance
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    nav_msgs
    tiago_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/nmpc_controller
  scripts/plotter
  scripts/send_desired_target_position
  scripts/object_detection
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
