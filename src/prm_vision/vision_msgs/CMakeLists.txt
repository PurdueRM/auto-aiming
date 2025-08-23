cmake_minimum_required(VERSION 3.5)
project(vision_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyPoints.msg"
  "msg/YawPitch.msg"
  "msg/PredictedArmor.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_generators)
ament_package()
