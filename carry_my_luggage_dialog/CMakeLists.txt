cmake_minimum_required(VERSION 3.0.2)
project(carry_my_luggage_dialog)


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
    roslint
    roscpp
    dialogflow_ros_msgs
    std_srvs
    std_msgs
    sound_play
    behaviortree_cpp_v3
    message_generation
    gb_dialog
    dialog_atlas
)


generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
	CATKIN_DEPENDS 
  roscpp 
  dialogflow_ros_msgs 
  message_runtime 
  behaviortree_cpp_v3
  gb_dialog
  std_msgs
  dialog_atlas
)


include_directories(
 
 include
 ${catkin_INCLUDE_DIRS}
 ${ZMQ_INCLUDE_DIRS}
 
)

add_library(${PROJECT_NAME}
src/carry_my_luggage_dialog/Get_Luggage.cpp
)



add_library(luggage_bt_node SHARED src/carry_my_luggage_dialog/Luggage.cpp)
list(APPEND plugin_libs
  luggage_bt_node)

target_link_libraries(luggage_bt_node ${PROJECT_NAME} )


foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


#add_executable(get_luggage_node src/get_luggage_node.cpp)
#target_link_libraries(get_luggage_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES}  )






roslint_cpp(
 src/carry_my_luggage_dialog/Get_Luggage.cpp
 src/carry_my_luggage_dialog/Luggage.cpp
 src/get_luggage_node
 
 include/carry_my_luggage_dialog/Get_Luggage.h
 include/carry_my_luggage_dialog/Luggage.h
 
 
)

