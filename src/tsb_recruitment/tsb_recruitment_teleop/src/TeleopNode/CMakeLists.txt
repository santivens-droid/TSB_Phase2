# Create the executable for node
add_executable(TeleopNode
  TeleopNode.cpp
)

# Make sure the node can find headers in the shared include folder
target_include_directories(TeleopNode PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(tsb_recruitment_interfaces REQUIRED)

# Link against dependencies
ament_target_dependencies(TeleopNode
  rclcpp
  tsb_recruitment_interfaces
)

# Install the executable
install(TARGETS TeleopNode
  DESTINATION lib/${PROJECT_NAME}
)