cmake_minimum_required(VERSION 3.8)
project(dofbot_info)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(dofbot_interface REQUIRED)

# Open re-implementation of Yahboom closed-source kinematics (KDL-based).
# Builds the two missing shared libs from the same source so existing
# link lines against libkin_srv keep working.
add_library(dofbot_kinematics SHARED src/dofbot_kinematics.cpp)
target_include_directories(dofbot_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dofbot_kinematics
  kdl_parser
)
target_link_libraries(dofbot_kinematics
  orocos-kdl
)

add_library(kin_srv SHARED src/dofbot_kinematics.cpp)
target_include_directories(kin_srv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kin_srv
  kdl_parser
)
target_link_libraries(kin_srv
  orocos-kdl
)

add_executable(kinemarics_dofbot src/kinemarics_dofbot.cpp)

target_include_directories(kinemarics_dofbot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinemarics_dofbot
  rclcpp
  kdl_parser
  std_msgs
  dofbot_interface
)

target_link_libraries(kinemarics_dofbot
  dofbot_kinematics
  orocos-kdl
)

install(TARGETS dofbot_kinematics kin_srv kinemarics_dofbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
