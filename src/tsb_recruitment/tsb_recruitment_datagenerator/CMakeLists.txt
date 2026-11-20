cmake_minimum_required(VERSION 3.8)
project(tsb_recruitment_datagenerator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Add ecah node
add_subdirectory(src/SP01DataGeneratorNode)

ament_package()
