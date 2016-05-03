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

  //Load the parameter containing the name of the joint controlling the
  // left wheel.
  if (!_sdf->HasElement("left_joint"))
  {
    std::cerr << "FRCBlueRobotPlugin::Load() No <left_joint> element"
              << std::endl;
    return;
  }

  std::string leftJoint = _sdf->Get<std::string>("left_joint");

  //Load the parameter containing the name of the joint controlling the
  // right wheel.
  if (!_sdf->HasElement("right_joint"))
  {
    std::cerr << "FRCBlueRobotPlugin::Load() No <right_joint> element"
              << std::endl;
    return;
  }

  std::string rightJoint = _sdf->Get<std::string>("right_joint");

  this->leftJoint = this->model->GetJoint(leftJoint);
  if (!this->leftJoint)
  {
    std::cerr << "FRCBlueRobotPlugin::Load() No joint found with name ["
              << leftJoint << "]" << std::endl;
    return;
  }

  this->rightJoint = this->model->GetJoint(rightJoint);
  if (!this->leftJoint)
  {
    std::cerr << "FRCBlueRobotPlugin::Load() No joint found with name ["
              << rightJoint << "]" << std::endl;
    return;
  }

  // ROS initialization.
  ros::M_string m;
  ros::init(m, "blue_team", ros::init_options::NoSigintHandler);
  this->n = new ros::NodeHandle();
  this->sub = this->n->subscribe(this->topic, 1, &FRCBlueRobotPlugin::OnData,
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

  //// Get linear velocity in world frame
  ignition::math::Vector3d linearVel = myPose.Rot().RotateVector(
    targetLinVel * ignition::math::Vector3d::UnitX);

  this->model->SetLinearVel(-linearVel);
  this->model->SetAngularVel(targetAngVel);
}

/////////////////////////////////////////////////
void FRCBlueRobotPlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  this->joyMsg = *_msg;
}
