cmake_minimum_required(VERSION 3.0.2)
project(robocup_navigation)

set(CMAKE_BUILD_TYPE Debug)

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
  message_generation
  move_base_msgs
  tf
)

add_service_files(
   DIRECTORY srv
   FILES Position.srv
)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp std_msgs 
  behaviortree_cpp_v3
  actionlib_msgs 
  move_base_msgs 
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(get_position_bt_node SHARED src/robocup_navigation/GetPosition.cpp)
add_library(move_bt_node SHARED src/robocup_navigation/Move.cpp)
list(APPEND plugin_libs
  get_position_bt_node
  move_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(move_node src/move_node.cpp)
add_executable(position_server_node src/position_server_node.cpp)

target_link_libraries(move_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
target_link_libraries(position_server_node ${catkin_LIBRARIES})

roslint_cpp(
  src/move_node.cpp
  src/position_server_node.cpp
  include/robocup_navigation/GetPosition.h src/robocup_navigation/GetPosition.cpp
  include/robocup_navigation/Move.h src/robocup_navigation/Move.cpp
)