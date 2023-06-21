cmake_minimum_required(VERSION 3.5)
project(keyboard_plugin)

find_package(gazebo REQUIRED)
find_package(ros2 REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED /src/keyboard_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${ros2_LIBRARIES} ${std_msgs_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${ros2_DEPENDENCIES} ${std_msgs_DEPENDENCIES})
