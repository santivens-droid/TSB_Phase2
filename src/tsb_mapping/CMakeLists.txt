cmake_minimum_required(VERSION 3.8)
project(tsb_mapping)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mapping_node src/mapping_node.cpp)
target_compile_features(mapping_node PUBLIC cxx_std_17)
target_include_directories(mapping_node PRIVATE include)
ament_target_dependencies(mapping_node rclcpp sensor_msgs nav_msgs)

install(TARGETS mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Testes matematicos sem dependencias ROS 2 adicionais.
if(BUILD_TESTING)
  enable_testing()
  add_executable(grid_core_test tests/grid_core_test.cpp)
  target_compile_features(grid_core_test PUBLIC cxx_std_17)
  target_include_directories(grid_core_test PRIVATE include)
  add_test(NAME grid_core_test COMMAND grid_core_test)
endif()

ament_package()
