# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(f710Driver)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs geometry_msgs genmsg carla_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs carla_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/talker.py
  scripts/driver.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/driver
)

# %EndTag(FULLTEXT)%
