cmake_minimum_required(VERSION 3.8)
project(gen3_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


add_executable(gen3_reset src/gen3_reset.cpp)
ament_target_dependencies(gen3_reset rclcpp trajectory_msgs sensor_msgs)

set(msg_files
  "msg/KeyFloat.msg"
  "msg/ParamDict.msg"
  "msg/Params.msg"
  "srv/ParamSkill.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES trajectory_msgs sensor_msgs
)


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

install(TARGETS
  gen3_reset
  DESTINATION lib/${PROJECT_NAME})


ament_package()

