cmake_minimum_required(VERSION 3.8)
project(tsb_recruitment_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/GPSPoint.msg"
  "msg/UTMPoint.msg"
  "msg/OperatingMode.msg"
  "msg/AgentCommand.msg"
  "msg/UIPopup.msg"
  "srv/SetOperatingMode.srv"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
