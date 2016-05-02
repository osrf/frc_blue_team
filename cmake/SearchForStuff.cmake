include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

#################################################
# Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#################################################
# Ignition math
find_package(ignition-math2 QUIET REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${IGNITION-MATH_LIBRARY_DIRS})

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

# gtest apparently needs boost::system. Boost_LIBRARIES is added to the link
# line for tests in TestUtils.cmake.
include(FindBoost)
find_package(Boost REQUIRED system thread filesystem program_options)
