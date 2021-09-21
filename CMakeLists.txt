cmake_minimum_required(VERSION 3.0.2)
project(atak_bridge)

set(CMAKE_BUILD_TYPE Release)

# TODO update the cmake file and package.xml file to reflect dependencies and remove any reference to AI TF CPP code.
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES atak_bridge
  CATKIN_DEPENDS message_runtime #roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

install(DIRECTORY src
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    FILES_MATCHING PATTERN "*.py"
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ
    )

install(PROGRAMS
    src/takpak/mkcot.py
    src/takpak/takcot.py
    src/takpak/__init__.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/takpak
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
