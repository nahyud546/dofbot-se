cmake_minimum_required(VERSION 3.8)
project(cap_scene_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SceneObject.msg" "msg/SceneObjects.msg"
  "msg/ObjectState.msg" "msg/ObjectStates.msg"
  "msg/GraspCandidate.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
