cmake_minimum_required(VERSION 3.0.2)
project(navegacion_robocup)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs behaviortree_cpp_v3 actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(prueba_node SHARED src/navegacion_robocup/Prueba.cpp)

list(APPEND plugin_libs
  prueba_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(navegacion_node src/navegacion_node.cpp)
target_link_libraries(navegacion_node ${catkin_LIBRARIES})

roslint_cpp(
  src/navegacion_node.cpp
  src/navegacion_robocup/Prueba.cpp include/navegacion_robocup/Prueba.h
)