cmake_minimum_required(VERSION 3.5.0)
project(keyboard_controller)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(${YAML_CPP_INCLUDE_DIR})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(position_keyboard_control examples/position_keyboard_control.cpp)
ament_target_dependencies(position_keyboard_control
  rclcpp
  rclcpp_action
  std_msgs
  yaml-cpp
)
add_executable(multi_keyboard_control examples/multi_keyboard_control.cpp)
ament_target_dependencies(multi_keyboard_control
  rclcpp
  rclcpp_action
  std_msgs
  yaml-cpp
)

target_link_libraries(position_keyboard_control ${YAML_CPP_LIBRARIES})
target_link_libraries(multi_keyboard_control ${YAML_CPP_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()


## Install
install(
  TARGETS
    position_keyboard_control
    multi_keyboard_control
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
