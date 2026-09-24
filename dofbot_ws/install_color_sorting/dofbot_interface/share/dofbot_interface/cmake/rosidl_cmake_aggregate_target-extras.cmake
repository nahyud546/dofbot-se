# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target dofbot_interface::dofbot_interface
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${dofbot_interface_TARGETS}.
if(dofbot_interface_TARGETS AND NOT TARGET dofbot_interface::dofbot_interface)
  add_library(dofbot_interface::dofbot_interface INTERFACE IMPORTED)
  set_target_properties(dofbot_interface::dofbot_interface PROPERTIES
    INTERFACE_LINK_LIBRARIES "${dofbot_interface_TARGETS}")
endif()
