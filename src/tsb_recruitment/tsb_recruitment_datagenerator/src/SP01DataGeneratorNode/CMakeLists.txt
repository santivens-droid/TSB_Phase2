# Create the executable for node
add_executable(SP01DataGeneratorNode
  SP01DataGeneratorNode.cpp
)

# Make sure the node can find headers in the shared include folder
target_include_directories(SP01DataGeneratorNode PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)


# Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tsb_recruitment_interfaces REQUIRED)

# Link against dependencies
ament_target_dependencies(SP01DataGeneratorNode 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tsb_recruitment_interfaces
)


# Install the executable
install(TARGETS SP01DataGeneratorNode
  DESTINATION lib/${PROJECT_NAME}
)


