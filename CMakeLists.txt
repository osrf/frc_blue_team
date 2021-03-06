cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(blue_team)

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

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package()

catkin_add_env_hooks(99_blue_team_setup SHELLS sh)

# Install the operator computer script.
catkin_install_python(PROGRAMS scripts/blue_team_go
  DESTINATION bin)

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

# Create the libFRCBlueGripperPlugin.so library.
set(frc_blue_gripper_plugin_name FRCBlueGripperPlugin)
add_library(${frc_blue_gripper_plugin_name} src/FRCBlueGripperPlugin.cc)
target_link_libraries(${frc_blue_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
install(TARGETS ${frc_blue_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libFRCBlueCameraPlugin.so library.
set(frc_blue_camera_plugin_name FRCBlueCameraPlugin)
add_library(${frc_blue_camera_plugin_name} src/FRCBlueCameraPlugin.cc)
target_link_libraries(${frc_blue_camera_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
install(TARGETS ${frc_blue_camera_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libFRCBlueAckermannDrivePlugin.so library.
set(frc_blue_ackermann_drive_plugin_name FRCBlueAckermannDrivePlugin)
add_library(${frc_blue_ackermann_drive_plugin_name} src/FRCBlueAckermannDrivePlugin.cc)
target_link_libraries(${frc_blue_ackermann_drive_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

install(TARGETS ${frc_blue_ackermann_drive_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libFRCBlueDiffDrivePlugin.so library.
set(frc_blue_diff_drive_plugin_name FRCBlueDiffDrivePlugin)
add_library(${frc_blue_diff_drive_plugin_name} src/FRCBlueDiffDrivePlugin.cc)
target_link_libraries(${frc_blue_diff_drive_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

install(TARGETS ${frc_blue_diff_drive_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libFRCBlueScorerPlugin.so library.
set(frc_blue_scorer_plugin_name FRCBlueScorerPlugin)
add_library(${frc_blue_scorer_plugin_name} src/FRCBlueScorerPlugin.cc)
target_link_libraries(${frc_blue_scorer_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

install(TARGETS ${frc_blue_scorer_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
