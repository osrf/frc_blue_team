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
#include "frc/FRCBlueRobotPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FRCBlueRobotPlugin)

/////////////////////////////////////////////////
FRCBlueRobotPlugin::~FRCBlueRobotPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FRCBlueRobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FRCBlueRobotPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FRCBlueRobotPlugin _sdf pointer is NULL");
  this->model = _model;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FRCBlueRobotPlugin::Update, this, _1));

  // ToDo: Load the parameter containing the joystick topic and subscribe to it.
  // Read the <topic> element.
  if (!_sdf->HasElement("topic"))
  {
    std::cerr << "FRCBlueRobotPlugin::Load() No <topic> element" << std::endl;
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
  this->sub = this->n->subscribe(this->topic, 1000, &FRCBlueRobotPlugin::OnData,
    this);
}

/////////////////////////////////////////////////
void FRCBlueRobotPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  ros::spinOnce();

  if (this->joyMsg.axes.empty())
    return;

  // Process last message.
  ignition::math::Vector3d targetLinVel(this->joyMsg.axes.at(1), 0.0, 0.0);
  math::Vector3 targetAngVel(0.0, 0.0, this->joyMsg.axes.at(0));

  auto myPose = this->model->GetWorldPose().Ign();

  // Get linear velocity in world frame
  ignition::math::Vector3d linearVel = myPose.Rot().RotateVector(
      targetLinVel * ignition::math::Vector3d::UnitX);

  this->model->SetLinearVel(linearVel);
  this->model->SetAngularVel(targetAngVel);
}

/////////////////////////////////////////////////
void FRCBlueRobotPlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  std::cout << "Joy received" << std::endl;
  this->joyMsg = *_msg;
}
