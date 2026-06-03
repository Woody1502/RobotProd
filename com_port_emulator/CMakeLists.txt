cmake_minimum_required(VERSION 3.10)
project(com_port_emulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(com_port_emulator_node src/com_port_emulator.cpp)

target_include_directories(com_port_emulator_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(com_port_emulator_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(TARGETS
  com_port_emulator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()