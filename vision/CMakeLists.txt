cmake_minimum_required(VERSION 2.8.3)
project(vision)

set (CMAKE_CXX_STANDARD 17)

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
  sensor_msgs
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
  pcl_conversions
  pcl_ros
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  fsm_visual_behavior
)

add_message_files(
  FILES 
  bbx_info.msg
  clothes.msg
  ropa_hsv.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
CATKIN_DEPENDS 
  roscpp 
  sensor_msgs
  std_msgs 
  behaviortree_cpp_v3 
  actionlib_msgs 
  move_base_msgs 
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_runtime
  fsm_visual_behavior
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(detect_operator_bt_node SHARED src/vision/DetectOperator.cpp)
add_library(detect_person_bt_node SHARED src/vision/DetectPerson.cpp)
list(APPEND plugin_libs
  detect_operator_bt_node
  detect_person_bt_node
)


foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(LINK_LIBS
  fsm_visual_behavior
  ${catkin_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(nodo_bbx_msg_publisher_advanced src/nodo_bbx_msg_publisher_advanced.cpp)
target_link_libraries(nodo_bbx_msg_publisher_advanced ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(nodo_bbx_msg_publisher_advanced vision_generate_messages_cpp)

add_executable(operator_tf_node src/operator_tf_node.cpp)
target_link_libraries(operator_tf_node ${catkin_LIBRARIES})

roslint_cpp(
  src/nodo_bbx_msg_publisher_advanced.cpp
  src/operator_tf_node.cpp
  include/vision/DetectPerson.h src/vision/DetectPerson.cpp
  include/vision/DetectOperator.h src/vision/DetectOperator.cpp
)