cmake_minimum_required(VERSION 2.8.3)
project(stageroscamcam)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp rostest sensor_msgs std_msgs tf)

find_package(Boost REQUIRED COMPONENTS system thread)

#find_package(FLTK REQUIRED)

#set(Stage_DIR cmake)
#find_package(stage_ros REQUIRED)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
#  ${FLTK_INCLUDE_DIRS}
#  ${Stage_INCLUDE_DIRS}
#)

#catkin_package(
#  INCLUDE_DIRS ${Stage_INCLUDE_DIRS}
#  LIBRARIES stageroscamcam ${Stage_LIBRARIES}
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS Boost
#)

# Declare a cpp executable
#add_executable(stageroscam src/stageroscam.cpp)
#target_link_libraries(stageroscam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${FLTK_LIBRARIES} ${Stage_LIBRARIES})
#if(catkin_EXPORTED_TARGETS)
#	add_dependencies(stageroscam ${catkin_EXPORTED_TARGETS})
#endif()

## Install

#install(PROGRAMS scripts/upgrade-world.sh
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(TARGETS stageroscam
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY rviz
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY world
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Tests

#add_rostest(test/hztest.xml)
