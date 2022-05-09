// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/package.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fsm_robocup");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;//creo el arboll
  BT::SharedLibrary loader;

  //registro los nodos los cuales añado y compilo como librerias en el cmakelists. Ademas en la clase de cada nodo al final, pones el metodo para que se 
  //identifique el nombre del nodo a su clase
  
  factory.registerFromPlugin(loader.getOSName("asr_move2_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_robocup_turn_node"));
  factory.registerFromPlugin(loader.getOSName("asr_robocup_person_detected_node"));
  factory.registerFromPlugin(loader.getOSName("asr_robocup_follow_person_node"));
  factory.registerFromPlugin(loader.getOSName("asr_robocup_is_bumped_node"));

  auto blackboard = BT::Blackboard::create();//creo la blackboard

  blackboard->set("object", "cup");
  std::string pkgpath = ros::package::getPath("fsm_robocup");//buscas el behavior tree creado con xml
  std::string xml_path_pkg;
  //n.getParam("xml_path_pkg", xml_path_pkg);
  xml_path_pkg = "/behavior_trees_xml/prueba.xml";
  std::string xml_path = pkgpath + xml_path_pkg;

  std::cerr << xml_path << "hola";
  BT::Tree tree = factory.createTreeFromFile(xml_path, blackboard);//creas el arbol
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(20);

  bool finish = false;
  while (ros::ok() && !finish)
  {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  blackboard->set("object", "cup");
  pkgpath = ros::package::getPath("fsm_robocup");
  xml_path_pkg = "/behavior_trees_xml/navigation.xml";
  xml_path = pkgpath + xml_path_pkg;
  tree = factory.createTreeFromFile(xml_path, blackboard);//creas el arbol

  while (ros::ok())
  {
    tree.rootNode()->executeTick();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
