cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(tsb_recruitment_bringup)

find_package(ament_cmake REQUIRED)

#============================================================================
# Resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
