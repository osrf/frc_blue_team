# Non-erb world files
set (world_files
  FRCblue.world
)

# Install all the world files
if (${gazebo_VERSION_MAJOR} GREATER 6)
  install(FILES ${world_files}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}/worlds)
else()
  install(FILES ${world_files}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/worlds)
endif()
