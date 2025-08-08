cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(controller_node src/controller.cpp)
ament_target_dependencies(controller_node rclcpp)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
