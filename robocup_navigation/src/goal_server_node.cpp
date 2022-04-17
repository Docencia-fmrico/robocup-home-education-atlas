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

#include "robocup_navigation/Goal.h"
#include "tf/transform_datatypes.h"
#include "robocup_navigation/enum_pos.h"

class GoalServer
{
private:
  static const int MAX_GUESTS = 3;
  int guests[MAX_GUESTS];
  int n_guests_;
  int guest_;

public:
  GoalServer() : n_guests_(0), guest_(0)
  { 
    srand(time(NULL));
    
    for (int i = 0; i < n_guests_; i++)
            guests[i] = 0;
  }

  bool goal(robocup_navigation::Goal::Request &req, robocup_navigation::Goal::Response &res)
  {
    int pos = req.position;

    switch (pos)
    {
      case INIT_POS:
        res.goal.position.x = 2.4;
        res.goal.position.y = 4.6;
        res.goal.position.z = 0.0;
        res.goal.orientation = tf::createQuaternionMsgFromYaw(M_PI);

        break;
      case OP_POS:
        res.goal.position.x = -3.7;
        res.goal.position.y = 4.0;
        res.goal.position.z = 0.0;
        res.goal.orientation = tf::createQuaternionMsgFromYaw(M_PI);

        break;
      case GUEST_POS:

        if (n_guests_ == 0)
        {
          bool repeat = true;
          do 
          {
            guest_ = rand() % GUEST_3 + GUEST_1;
            ROS_INFO("traza-------- %i", guest_);
            // checks if the number of the guest is not greater than the last guest
            repeat = guest_ > GUEST_3;
          }
          while (repeat);
          guests[n_guests_] = guest_;
          n_guests_++;
        }

        else if (n_guests_ < MAX_GUESTS)
        {
          bool repeated = true;
          do 
          {
            guest_ = rand() % GUEST_3 + GUEST_1;
            ROS_INFO("traza-------- %i", guest_);
            // checks that the guest is not repeated
            for (int i = 0; i < n_guests_; i++)
            {
              if (guests[i] == guest_ || guest_ > GUEST_3) 
                break;
              repeated = guests[i] != guest_ && i != n_guests_-1;
            }
          }
          while (repeated);
          
          guests[n_guests_] = guest_;
          n_guests_++;
        }

        else
        {
          n_guests_ = 0;

          for (int i = 0; i < n_guests_; i++)
            guests[i] = 0;
        }

        break;
    }

    switch(guest_)
    {
      case GUEST_1:
        res.goal.position.x = 1.5;
        res.goal.position.y = -1.5;
        res.goal.position.z = 0.0;
        res.goal.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
        ROS_INFO("Guest 1");

        break;
      case GUEST_2:
        res.goal.position.x = 2.5;
        res.goal.position.y = -0.5;
        res.goal.position.z = 0.0;
        res.goal.orientation = tf::createQuaternionMsgFromYaw(0.0);
        ROS_INFO("Guest 2");

        break;
      case GUEST_3:
        res.goal.position.x = 0.9;
        res.goal.position.y = 0.4;
        res.goal.position.z = 0.0;
        res.goal.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
        ROS_INFO("Guest 3");

        break;
    }
    
    ROS_INFO("request: %i", req.position);
    ROS_INFO("sending back response: [%lf]", res.goal.position.x);
    return true;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_server");
  ros::NodeHandle n;

  // define el nombre del servicio
  GoalServer s;
  ros::ServiceServer service = n.advertiseService("goal", &GoalServer::goal, &s);
  ROS_INFO("Ready to send goal.");
  ros::spin();

  return 0;
}