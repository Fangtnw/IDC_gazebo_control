/*
2  * Copyright (C) 2016 Open Source Robotics Foundation
3  *
4  * Licensed under the Apache License, Version 2.0 (the "License");
5  * you may not use this file except in compliance with the License.
6  * You may obtain a copy of the License at
7  *
8  * http://www.apache.org/licenses/LICENSE-2.0
9  *
10  * Unless required by applicable law or agreed to in writing, software
11  * distributed under the License is distributed on an "AS IS" BASIS,
12  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
13  * See the License for the specific language governing permissions and
14  * limitations under the License.
15  *
16 */

#ifndef GAZEBO_PLUGINS_KEYSTOJOINTSPLUGIN_HH_
#define GAZEBO_PLUGINS_KEYSTOJOINTSPLUGIN_HH_

#include <string>
#include <vector>
#include <ignition/transport/Node.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
 
namespace gazebo
{
struct KeyInfo
{
// cppcheck-suppress unusedStructMember
int key;

physics::JointPtr joint;

std::string type;

// cppcheck-suppress unusedStructMember
double scale;
};

class GAZEBO_VISIBLE KeysToJointsPlugin : public ModelPlugin
  {
  public: KeysToJointsPlugin();

  public: ~KeysToJointsPlugin();

  // Documentation inherited
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
 
  private: void OnKeyPress(ConstAnyPtr &_msg);

  private: std::vector<KeyInfo> keys;

 private: physics::ModelPtr model;

 private: transport::NodePtr node;
 
 private: transport::SubscriberPtr keyboardSub;

 // Place ignition::transport objects at the end of this file to
 // guarantee they are destructed first.

 private: ignition::transport::Node nodeIgn;
  };
}
#endif
