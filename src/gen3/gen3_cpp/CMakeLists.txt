cmake_minimum_required(VERSION 3.8)
project(gen3_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(gen3_reset src/gen3_reset.cpp)
ament_target_dependencies(gen3_reset rclcpp trajectory_msgs)

install(TARGETS
  gen3_reset
  DESTINATION lib/${PROJECT_NAME})

ament_package()

