cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(frc_blue_team)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

catkin_package()

# Create the libFRCBlueRobotPlugin.so library.
set(frc_blue_robot_plugin_name FRCBlueRobotPlugin)
add_library(${frc_blue_robot_plugin_name} src/FRCBlueRobotPlugin.cc)
target_link_libraries(${frc_blue_robot_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
install(TARGETS ${frc_blue_robot_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install all the world files
set(world_files worlds/FRCblue.world)
if (${gazebo_VERSION_MAJOR} GREATER 6)
  install(FILES ${world_files}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}/worlds)
else()
  install(FILES ${world_files}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/worlds)
endif()
