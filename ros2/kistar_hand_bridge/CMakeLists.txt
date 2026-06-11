cmake_minimum_required(VERSION 3.8)
project(kistar_hand_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(kistar_hand_bridge_node
  src/kistar_hand_bridge_node.cpp
  ../../src/shm.cpp
)

target_include_directories(kistar_hand_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../inc
)

target_compile_features(kistar_hand_bridge_node PRIVATE cxx_std_17)

ament_target_dependencies(kistar_hand_bridge_node
  rclcpp
  std_msgs
  trajectory_msgs
)

install(TARGETS
  kistar_hand_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
