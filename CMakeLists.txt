cmake_minimum_required(VERSION 2.8.3)
project(fsm_robocup)

# set(CMAKE_BUILD_TYPE Debug)

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
  OpenCV
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
  FILES 
  bbx_info.msg
  ropa_hsv.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  behaviortree_cpp_v3 
  actionlib_msgs 
  move_base_msgs 
  cv_bridge
  image_transport
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(asr_move2_bt_node SHARED src/behavior_trees/Move.cpp)
list(APPEND plugin_libs
  asr_move2_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nodo_behavior_trees_robocup 
  src/nodo_behavior_trees_robocup.cpp
)
target_link_libraries(nodo_behavior_trees_robocup ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(nodo_bbx_msg_publisher_advanced src/nodo_bbx_msg_publisher_advanced.cpp)
target_link_libraries(nodo_bbx_msg_publisher_advanced ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(nodo_bbx_msg_publisher_advanced fsm_robocup_generate_messages_cpp)

roslint_cpp(
  src/nodo_behavior_trees_robocup.cpp
)
