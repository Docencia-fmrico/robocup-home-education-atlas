// Copyright 2022 Team Atlas
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

#include "time.h"
#include "ros/ros.h"

#include "enum_pos.h"
#include "movement/Position.h"
#include "tf/transform_datatypes.h"

class PositionServer
{
public:
  PositionServer(){ }

  bool position(movement::Position::Request &req, movement::Position::Response &res)
  {
    int pos = req.goal;

    switch (pos)
    {
      case INIT_POS:
        res.pos.position.x = 0.0;
        res.pos.position.y = 0.0;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(M_PI);

        break;
      case OP_POS:
        res.pos.position.x = 5.0;
        res.pos.position.y = -1.4;
        res.pos.position.z = 0.0;
        res.pos.orientation.x = 0.0;
        res.pos.orientation.y = 0.0;
        res.pos.orientation.z = 0.0;
        res.pos.orientation.w = 1.0;

        break;
      case GUESTS_POS:
        res.pos.position.x = 3.3;
        res.pos.position.y = 3.6;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

        break;
      case GUEST1:
        res.pos.position.x = 2.9;
        res.pos.position.y = 1.8;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

        break;
      case GUEST2:
        res.pos.position.x = 1.2;
        res.pos.position.y = 2.3;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);

        break;
      case GUEST3:
        res.pos.position.x = 0.8;
        res.pos.position.y = 3.9;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(M_PI);

        break;
      case GUEST4:
        res.pos.position.x = 1.5;
        res.pos.position.y = 6.2;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(M_PI);

        break;
      case GUEST5:
        res.pos.position.x = 2.2;
        res.pos.position.y = 7.4;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

        break;
      case GUEST6:
        res.pos.position.x = 4.0;
        res.pos.position.y = 6.4;
        res.pos.position.z = 0.0;
        res.pos.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

        break;
    }
    
    ROS_INFO("request: %i", req.goal);
    ROS_INFO("sending back response: [%lf]", res.pos.position.x);
    return true;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_server");
  ros::NodeHandle n;

  // define el nombre del servicio
  PositionServer s;
  ros::ServiceServer service = n.advertiseService("position", &PositionServer::position, &s);
  ROS_INFO("Ready to send position.");
  ros::spin();

  return 0;
}