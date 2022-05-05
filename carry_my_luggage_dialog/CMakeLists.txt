cmake_minimum_required(VERSION 3.0.2)
project(carry_my_luggage_dialog)


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
project(carry_my_luggage_dialog)

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
    dialog_atlas
)


#add_message_files(
#  FILES 
 
#)


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
  dialog_atlas
)


include_directories(
 
 ${catkin_INCLUDE_DIRS}
 ${ZMQ_INCLUDE_DIRS}
 include
)


add_library(${PROJECT_NAME}
  src/carry_my_luggage_dialog/Get_Luggage.cpp

 )



add_executable(get_luggage_node src/get_luggage_node.cpp)
add_dependencies(get_luggage_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_luggage_node ${catkin_LIBRARIES} carry_my_luggage_dialog)





roslint_cpp(
 src/carry_my_luggage_dialog/Get_Luggage.cpp
 
 
 
)

