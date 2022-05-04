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


#include <ros/ros.h>

#include "dialog_atlas/Application.h"
#include "dialog_atlas/activate_msg.h"



namespace dialog_atlas
{
    Application::Application(): nh_()
    {
        pub_ = nh_.advertise<dialog_atlas::activate_msg>("/activate_dialog",1);
    }


    void Application::activate(bool activate_dialog)
    {
        dialog_atlas::activate_msg msg;
        msg.activate = activate_dialog;
        pub_.publish(msg);
    }



}








