cmake_minimum_required(VERSION 3.0.2)
project(atak_bridge)

set(CMAKE_BUILD_TYPE Release)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PoseDescription.msg
   PoseDescriptionArray.msg
   PoseDescriptionStamped.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   atak_bridge
 )


#######################################
## Declare Catkin package 
#######################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime #roscpp rospy sensor_msgs
  DEPENDS 
)

include_directories(include 
    ${catkin_INCLUDE_DIRS} 
)

