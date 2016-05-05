/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <mutex>
#include <string>
#include <ignition/math/Helpers.hh>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "frc/FRCBlueGripperPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FRCBlueGripperPlugin)


/////////////////////////////////////////////////
FRCBlueGripperPlugin::~FRCBlueGripperPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FRCBlueGripperPlugin::Load(physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FRCBlueGripperPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FRCBlueGripperPlugin _sdf pointer is NULL");
  this->model = _model;

//  for (auto j : _model->GetJoints())
//    std::cerr << j->GetName() << std::endl;

  // diff drive params
  if (_sdf->HasElement("left"))
  {
    this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left")->Get<std::string>());
  }
  if (_sdf->HasElement("right"))
  {
    this->rightJoint = _model->GetJoint(
        _sdf->GetElement("right")->Get<std::string>());
  }
  if (_sdf->HasElement("up"))
  {
    this->upJoint = _model->GetJoint(
        _sdf->GetElement("up")->Get<std::string>());
  }


  if (!this->leftJoint || !this->rightJoint || !this->upJoint)
  {
    std::cerr << "Gripper joints not found! " << std::endl;
    return;
  }


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FRCBlueGripperPlugin::Update, this, _1));

  // ToDo: Load the parameter containing the joystick topic and subscribe to it.
  // Read the <topic> element.
  if (!_sdf->HasElement("topic"))
  {
    std::cerr << "FRCBlueGripperPlugin::Load() No <topic> element"
        << std::endl;
    return;
  }

  // Read the topic parameter.
  this->topic = _sdf->Get<std::string>("topic");
  std::cout << "[" << this->model->GetName() << "] listening Joystick commands"
            << " on ROS topic [" << this->topic << "]" << std::endl;

  // ROS initialization.
  ros::M_string m;
  ros::init(m, "blue_team", ros::init_options::NoSigintHandler);
  this->n = new ros::NodeHandle();
  this->sub = this->n->subscribe(this->topic, 1000,
      &FRCBlueGripperPlugin::OnData, this);
}

/////////////////////////////////////////////////
void FRCBlueGripperPlugin::Init()
{
}

/////////////////////////////////////////////////
void FRCBlueGripperPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  ros::spinOnce();

  if (this->joyMsg.buttons.empty())
    return;

  double grip = 0;
  double lift = 0;

  // ps controller
  if (this->joyMsg.buttons.size() > 12)
  {
    // X
    grip = this->joyMsg.buttons.at(14);
    // square
    lift = this->joyMsg.buttons.at(15);
  }
  // logitech wireless
  else
  {
    // A
    grip = this->joyMsg.buttons.at(1);
    // X
    lift = this->joyMsg.buttons.at(0);
  }

  if (grip)
  {
    this->leftJoint->SetForce(0, 10);
    this->rightJoint->SetForce(0, -10);
  }
  else
  {
    this->leftJoint->SetForce(0, -10);
    this->rightJoint->SetForce(0, 10);
  }

  if (lift)
  {
    double f = lift * 300;
    this->upJoint->SetForce(0, f);
  }
}

/////////////////////////////////////////////////
void FRCBlueGripperPlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  this->joyMsg = *_msg;
}
