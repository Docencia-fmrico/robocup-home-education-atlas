cmake_minimum_required(VERSION 3.0.2)
project(find_my_mates)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
  cv_bridge
  darknet_ros_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp std_msgs 
  behaviortree_cpp_v3
  actionlib_msgs 
  move_base_msgs 
  tf
  cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)


add_library(get_position_bt_node SHARED src/find_my_mates/GetPosition.cpp)
add_library(detect_guest_bt_node SHARED src/find_my_mates/DetectGuest.cpp)
add_library(move_bt_node SHARED src/find_my_mates/Move.cpp)
add_library(approach_operator_bt_node SHARED src/find_my_mates/ApproachOperator.cpp)
list(APPEND plugin_libs
  get_position_bt_node
  detect_guest_bt_node
  move_bt_node
  approach_operator_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(find_my_mates_node
  src/find_my_mates_node.cpp
)
target_link_libraries(find_my_mates_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/find_my_mates_node.cpp
  include/find_my_mates/GetPosition.h src/find_my_mates/GetPosition.cpp
  include/find_my_mates/DetectGuest.h src/find_my_mates/DetectGuest.cpp
  include/find_my_mates/Move.h src/find_my_mates/Move.cpp
  include/find_my_mates/ApproachOperator.h src/find_my_mates/ApproachOperator.cpp
)