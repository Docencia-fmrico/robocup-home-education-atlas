cmake_minimum_required(VERSION 3.0.2)
project(movement)

set(CMAKE_CXX_STANDARD 17)
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
  geometry_tf
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
  geometry_tf
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/PIDController.cpp)

add_library(approach_person_bt_node SHARED src/movement/ApproachPerson.cpp)
add_library(get_position_bt_node SHARED src/movement/GetPosition.cpp)
add_library(go_to_position_bt_node SHARED src/movement/GoToPosition.cpp)
add_library(turn_bt_node SHARED src/movement/Turn.cpp)
list(APPEND plugin_libs
  approach_person_bt_node
  get_position_bt_node
  go_to_position_bt_node
  turn_bt_node
)

target_link_libraries(approach_person_bt_node ${PROJECT_NAME})

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
  include/movement/ApproachPerson.h src/movement/ApproachPerson.cpp
  include/movement/GetPosition.h src/movement/GetPosition.cpp
  include/movement/GoToPosition.h src/movement/GoToPosition.cpp
  include/movement/Turn.h src/movement/Turn.cpp
)