include (${project_cmake_dir}/Utils.cmake)

set (sources
  FRCBlueRobotPlugin.cc
)

# Tests
set (gtest_sources

)

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES
                            GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR}/
                    ${CMAKE_BINARY_DIR}/test/
                    ${catkin_INCLUDE_DIRS})

ign_build_tests(${gtest_sources})

# Create the libFRCBlueRobotPlugin.so library.
ign_add_library(${PROJECT_LIB_ROBOT_NAME}
                ${sources})
target_link_libraries(${PROJECT_LIB_ROBOT_NAME} ${catkin_LIBRARIES})
ign_install_library(${PROJECT_LIB_ROBOT_NAME})
