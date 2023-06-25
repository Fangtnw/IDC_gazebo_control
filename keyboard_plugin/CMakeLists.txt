cmake_minimum_required(VERSION 3.0)
project(keyboard_plugin)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

find_package(gazebo QUIET)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS} include/keyboard_plugin)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(KeyboardGUIPlugin MODULE src/KeyboardGUIPlugin.cc)
target_link_libraries(KeyboardGUIPlugin ${GAZEBO_LIBRARIES} Qt5::Core)
