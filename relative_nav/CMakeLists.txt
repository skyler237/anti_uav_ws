cmake_minimum_required(VERSION 2.8.3)
project(relative_nav)

find_package(catkin REQUIRED 
  roscpp 
  rospy 
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_message_files(
  FILES
  FilterState.msg
  DesiredVector.msg
  DesiredState.msg
  Edge.msg
  NodeInfo.msg
  VOUpdate.msg
  Keyframe.msg
  Command.msg
  Waypoint.msg
  Path.msg
  Voltage.msg
  Goal.msg
  NodePath.msg
  Match.msg
  GPS.msg
  Point2D.msg
  Line.msg
  Obstacles.msg
  DEBUG_Controller.msg
  Snapshot.msg
)

add_service_files(
  FILES
  Transform.srv
  HighLevelPath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES relative_nav
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs sensor_msgs nav_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
add_library(relative_nav
   src/eigen.cpp
   src/geometry.cpp
   src/control.cpp
   src/simple_pid.cpp
   include/relative_nav/control.h
   include/relative_nav/eigen.h
   include/relative_nav/geometry.h
   include/relative_nav/params.h
   include/relative_nav/lock.h
   include/relative_nav/simple_pid.h
   include/relative_nav/strings.h
 )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(relative_nav relative_nav_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(relative_nav ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS relative_nav
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.h"
            PATTERN "*.svn" EXCLUDE 
)
