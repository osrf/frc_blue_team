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
#include "frc/FRCBlueScorerPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FRCBlueScorerPlugin)

/////////////////////////////////////////////////
InternalJointController::InternalJointController(const physics::JointPtr _joint,
  const double _p, const double _d,
  const double _lowerTarget, const double _upperTarget,
  const int _buttonIndex)
: joint(_joint),
  lowerTarget(_lowerTarget),
  upperTarget(_upperTarget),
  target(_lowerTarget),
  buttonIndex(_buttonIndex)
{
  this->pid.SetPGain(_p);
  this->pid.SetDGain(_d);
}

/////////////////////////////////////////////////
void InternalJointController::Update(const sensor_msgs::Joy &_joyMsg)
{
  if (!this->joint)
    return;

  if (static_cast<int>(_joyMsg.buttons.size()) <= this->buttonIndex)
  {
    //std::cerr << "Unexpected axis size" << std::endl;
    return;
  }

  auto now = gazebo::physics::get_world()->GetSimTime();

  // Decide if we have to change the target.
  double currentValue = _joyMsg.buttons.at(this->buttonIndex);
  if ((ignition::math::equal(currentValue, 1.0)) &&
      (now - this->lastToggleTime > common::Time(0.5)))
  {
    this->Toggle();
    this->lastToggleTime = now;
  }

  auto dt = now - this->lastControllerUpdate;
  auto current = this->joint->GetAngle(0);
  auto error = current - this->target;
  double output = this->pid.Update(error.Radian(), dt);

  if ((ignition::math::equal(target, this->lowerTarget)) &&
      (this->joint->GetName().find("_beam_lift") != std::string::npos))
  {
    output = std::max(std::min(output, 1000.0), 0.0);
  }
  else
    output = std::max(std::min(output, 1000.0), -1000.0);

  this->joint->SetForce(0, output);

  this->lastControllerUpdate = now;
}

/////////////////////////////////////////////////
void InternalJointController::Toggle()
{
  if (ignition::math::equal(this->target, this->lowerTarget))
    this->target = this->upperTarget;
  else
    this->target = this->lowerTarget;
}

/////////////////////////////////////////////////
FRCBlueScorerPlugin::~FRCBlueScorerPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FRCBlueScorerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FRCBlueScorerPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FRCBlueScorerPlugin _sdf pointer is NULL");
  this->model = _model;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FRCBlueScorerPlugin::Update, this, _1));

  // ToDo: Load the parameter containing the joystick topic and subscribe to it.
  // Read the <topic> element.
  if (!_sdf->HasElement("topic"))
  {
    std::cerr << "FRCBlueScorerPlugin::Load() No <topic> element" << std::endl;
    return;
  }

  // Read the topic parameter.
  this->topic = _sdf->Get<std::string>("topic");
  std::cout << "[" << this->model->GetName() << "] listening Joystick commands"
            << " on ROS topic [" << this->topic << "]" << std::endl;

  // Load the <joint> parameters.
  for (auto jointElem = _sdf->GetElement("joint"); jointElem;
    jointElem = jointElem->GetNextElement("joint"))
  {
    // Mandatory fields.
    if (!jointElem->HasElement("name"))
    {
      std::cerr << "Unable to find <name> element" << std::endl;
      continue;
    }

    std::string name =  jointElem->Get<std::string>("name");
    std::cout << "Name [" << name << "]" << std::endl;

    if (!jointElem->HasElement("lower_limit"))
    {
      std::cerr << "Unable to find <lower_limit> element" << std::endl;
      continue;
    }

    double lowerLimit =  jointElem->Get<double>("lower_limit");
    std::cout << "Lower limit [" << lowerLimit << "]" << std::endl;

    if (!jointElem->HasElement("upper_limit"))
    {
      std::cerr << "Unable to find <upper_limit> element" << std::endl;
      continue;
    }

    double upperLimit =  jointElem->Get<double>("upper_limit");
    std::cout << "Upper limit [" << upperLimit << "]" << std::endl;

    if (!jointElem->HasElement("button_index"))
    {
      std::cerr << "Unable to find <button_index> element" << std::endl;
      continue;
    }

    int buttonIndex =  jointElem->Get<double>("button_index");
    std::cout << "Button index [" << buttonIndex << "]" << std::endl;

    // Optional fields.
    if (!jointElem->HasElement("p_gain"))
    {
      std::cerr << "Unable to find <p_gain> element" << std::endl;
      continue;
    }
    double pGain =  jointElem->Get<double>("p_gain");
    std::cout << "P gain [" << pGain << "]" << std::endl;

    if (!jointElem->HasElement("d_gain"))
    {
      std::cerr << "Unable to find <d_gain> element" << std::endl;
      continue;
    }

    double dGain =  jointElem->Get<double>("d_gain");
    std::cout << "D gain [" << dGain << "]" << std::endl;

    // Add a new joint controller.
    auto jointPtr = this->model->GetJoint(name);
    if (!jointPtr)
    {
      std::cerr << "Unable to find joint [" << name << "]" << std::endl;
      continue;
    }

    this->controllers.push_back(new InternalJointController(jointPtr,
      pGain, dGain, lowerLimit, upperLimit, buttonIndex));
  }

  // ROS initialization.
  ros::M_string m;
  ros::init(m, "blue_team", ros::init_options::NoSigintHandler);
  this->n = new ros::NodeHandle();
  this->sub = this->n->subscribe(this->topic, 1000, &FRCBlueScorerPlugin::OnData,
    this);
}

/////////////////////////////////////////////////
void FRCBlueScorerPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  ros::spinOnce();

   // Update all the controllers.
  for (auto controller : this->controllers)
    controller->Update(this->joyMsg);
}

/////////////////////////////////////////////////
void FRCBlueScorerPlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  this->joyMsg = *_msg;
}
