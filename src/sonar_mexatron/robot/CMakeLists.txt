cmake_minimum_required(VERSION 3.0.2)
project(sonar_mexatron)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  rospy
  std_msgs
)


# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES robot
# #  CATKIN_DEPENDS roscpp rospy std_msgs
# #  DEPENDS system_lib
# )

include_directories(

  ${catkin_INCLUDE_DIRS}
)

