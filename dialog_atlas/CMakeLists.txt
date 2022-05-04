# Copyright 2022 Team Atlas
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



cmake_minimum_required(VERSION 3.0.2)
project(dialog_atlas)

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
    actionlib
    actionlib_msgs
    behaviortree_cpp_v3
    message_generation
    gb_dialog
)


add_message_files(
  FILES 
  get_name.msg
  activate_msg.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
	CATKIN_DEPENDS 
  roscpp 
  dialogflow_ros_msgs 
  actionlib_msgs 
  message_runtime 
  behaviortree_cpp_v3
  gb_dialog
  std_msgs
)


include_directories(
 
 ${catkin_INCLUDE_DIRS}
 ${ZMQ_INCLUDE_DIRS}
 include
)


add_library(${PROJECT_NAME}
  src/dialog_atlas/Get_Name.cpp
  src/dialog_atlas/ProvideInfo.cpp
  src/dialog_atlas/Activation.cpp
  src/dialog_atlas/Name.cpp

 )



foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_executable(pub_name_node src/dialog_topic.cpp)
target_link_libraries(pub_name_node ${catkin_LIBRARIES} dialog_atlas)
add_dependencies(pub_name_node dialog_atlas_generate_messages_cpp)


add_executable(pub_activate_node src/activate_topic.cpp)
target_link_libraries(pub_activate_node ${catkin_LIBRARIES} dialog_atlas)
add_dependencies(pub_activate_node dialog_atlas_generate_messages_cpp)

add_executable(provide_info_node src/provide_info_node.cpp)
target_link_libraries(provide_info_node ${catkin_LIBRARIES} dialog_atlas)




roslint_cpp(
 src/dialog_atlas/Get_Name.cpp
 
 src/get_name_node.cpp
 src/provide_info_node.cpp
 src/dialog_atlas/ProvideInfo.cpp
 src/dialog_atlas/Activation.cpp
 src/dialog_atlas/Name.cpp


 include/dialog_atlas/Activation.h 
 include/dialog_atlas/ProvideInfo.h 
 include/dialog_atlas/Name.h 

 
 
)

