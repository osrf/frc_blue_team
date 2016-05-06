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
#include "frc/FRCBlueAckermannDrivePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FRCBlueAckermannDrivePlugin)

// Used for left/right wheel.
enum {RIGHT, LEFT};


/////////////////////////////////////////////////
FRCBlueAckermannDrivePlugin::~FRCBlueAckermannDrivePlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FRCBlueAckermannDrivePlugin::Load(physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FRCBlueAckermannDrivePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FRCBlueAckermannDrivePlugin _sdf pointer is NULL");
  this->model = _model;

  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
  this->wheelSeparation = 1.0;
  this->wheelRadius = 1.0;

  // drive params
  if (_sdf->HasElement("left_joint"))
  {
    this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->Get<std::string>());
  }
  if (_sdf->HasElement("right_joint"))
  {
    this->rightJoint = _model->GetJoint(
        _sdf->GetElement("right_joint")->Get<std::string>());
  }

  if (_sdf->HasElement("front_left_joint"))
  {
    this->frontLeftJoint = _model->GetJoint(
      _sdf->GetElement("front_left_joint")->Get<std::string>());
  }
  if (_sdf->HasElement("front_right_joint"))
  {
    this->frontRightJoint = _model->GetJoint(
        _sdf->GetElement("front_right_joint")->Get<std::string>());
  }

  // steering params
  if (_sdf->HasElement("left_steering_joint"))
  {
    this->steeringJoints[LEFT] = _model->GetJoint(
      _sdf->GetElement("left_steering_joint")->Get<std::string>());
  }
  if (_sdf->HasElement("right_steering_joint"))
  {
    this->steeringJoints[RIGHT] = _model->GetJoint(
        _sdf->GetElement("right_steering_joint")->Get<std::string>());
  }

  if (_sdf->HasElement("torque"))
  {
    this->torque = _sdf->Get<double>("torque");
  }
  else
    this->torque = 0.01;

  if (_sdf->HasElement("turn_rate"))
  {
    this->turnTorque = _sdf->Get<double>("turn_rate");
  }
  else
    this->turnTorque = 0.01;

  if (!this->leftJoint || !this->rightJoint)
    this->FindJoints();

  if (!this->leftJoint || !this->rightJoint)
  {
    gzerr << "left or right joint not found!" << std::endl;
    return;
  }


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FRCBlueAckermannDrivePlugin::Update, this, _1));

  // Read the <topic> element.
  if (!_sdf->HasElement("topic"))
  {
    std::cerr << "FRCBlueAckermannDrivePlugin::Load() No <topic> element"
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
      &FRCBlueAckermannDrivePlugin::OnData, this);
}

/////////////////////////////////////////////////
void FRCBlueAckermannDrivePlugin::Init()
{
  if (!this->leftJoint || !this->rightJoint)
    return;

  this->wheelSeparation =
      this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  ignition::math::Box bb = parent->GetBoundingBox().Ign();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.Size().Max() * 0.5;
}

/////////////////////////////////////////////////
void FRCBlueAckermannDrivePlugin::Update(const common::UpdateInfo & /*_info*/)
{
  ros::spinOnce();

  if (this->joyMsg.axes.empty())
    return;

  if (this->joyMsg.buttons.empty())
    return;

  double boostFactor = 1;
  // ps controller
  if (this->joyMsg.buttons.size() > 12)
    boostFactor = this->joyMsg.buttons.at(12) ? 2 : 1;
  // logitech wireless
  else
    boostFactor = this->joyMsg.buttons.at(3) ? 2 : 1;

  double steerAngle = this->joyMsg.axes.at(0);
  double vr = this->joyMsg.axes.at(1)* this->torque * boostFactor;

  double stepTime = 0.005; // ToDo: use actual step calculated from time diff
  double currentLeftAngle = this->steeringJoints[LEFT]->GetAngle(0).Radian();
    this->steeringJoints[LEFT]->SetVelocity(0, (steerAngle - currentLeftAngle) / stepTime);
  double currentRightAngle = this->steeringJoints[RIGHT]->GetAngle(0).Radian();
    this->steeringJoints[RIGHT]->SetVelocity(0, (steerAngle - currentRightAngle) / stepTime);
  this->steeringJoints[LEFT]->SetVelocityLimit(0, 0.5);
  this->steeringJoints[RIGHT]->SetVelocityLimit(0, 0.5);

  this->wheelSpeed[LEFT] =
      vr;// + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] =
      vr;// - va * this->wheelSeparation / 2.0;

  double leftVelDesired =
      (this->wheelSpeed[LEFT] / this->wheelRadius);
  double rightVelDesired =
      (this->wheelSpeed[RIGHT] / this->wheelRadius);

  /* 
  double currentLeftVel = this->leftJoint->GetVelocity(0);
  double currentRightVel = this->rightJoint->GetVelocity(0);
  double leftEffort = (leftVelDesired - currentLeftVel) / stepTime;
  double rightEffort = (rightVelDesired - currentRightVel) / stepTime;
  */

  this->leftJoint->SetVelocityLimit(0, 0.5);
  this->rightJoint->SetVelocityLimit(0, 0.5);
  this->leftJoint->SetForce(0, leftVelDesired);
  this->rightJoint->SetForce(0, rightVelDesired);

}

/////////////////////////////////////////////////
void FRCBlueAckermannDrivePlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  this->joyMsg = *_msg;
}

/////////////////////////////////////////////////
void FRCBlueAckermannDrivePlugin::FindJoints()
{
  // assumes the first two revolute joints are the ones connecting the
  // wheels to the chassis
  auto joints = this->model->GetJoints();
  if (joints.size() < 2u)
    return;

  physics::Joint_V revJoints;
  for (const auto &j : joints)
  {
    if (j->GetMsgType() == msgs::Joint::REVOLUTE)
      revJoints.push_back(j);
  }

  if (revJoints.size() < 2u)
    return;

  this->leftJoint = revJoints[0];
  this->rightJoint = revJoints[1];
}
