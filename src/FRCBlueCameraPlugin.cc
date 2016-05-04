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
#include "gazebo/math/Rand.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "frc/FRCBlueCameraPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FRCBlueCameraPlugin)

/////////////////////////////////////////////////
FRCBlueCameraPlugin::~FRCBlueCameraPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FRCBlueCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FRCBlueCameraPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FRCBlueCameraPlugin _sdf pointer is NULL");
  this->model = _model;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FRCBlueCameraPlugin::Update, this, _1));

  // ToDo: Load the parameter containing the joystick topic and subscribe to it.
  // Read the <topic> element.
  if (!_sdf->HasElement("topic"))
  {
    std::cerr << "FRCBlueCameraPlugin::Load() No <topic> element" << std::endl;
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
      &FRCBlueCameraPlugin::OnData, this);
}

/////////////////////////////////////////////////
void FRCBlueCameraPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  ros::spinOnce();

  if (this->joyMsg.axes.empty())
    return;

  double posStep = 0.001;
  double rotStep = 0.0005;


  double pan = 0;
  double tilt = 0;
  double left = 0;
  double right = 0;
  double down = 0;
  double up = 0;

  // ps controller
  if (this->joyMsg.buttons.size() > 11)
  {
    // right joy left right
    pan = this->joyMsg.axes.at(2);
    // right joy up down
    tilt = -this->joyMsg.axes.at(3);

    // l1
    left = this->joyMsg.buttons.at(10);
    // r1
    right = this->joyMsg.buttons.at(11);

    // l2
    down = this->joyMsg.buttons.at(8);
    // r2
    up = this->joyMsg.buttons.at(9);

  }
  // logitech
  else
  {
    // right joy left right
    pan = this->joyMsg.axes.at(3);
    // right joy up down
    tilt = -this->joyMsg.axes.at(4);

    // l1
    left = this->joyMsg.buttons.at(4);
    // r1
    right = this->joyMsg.buttons.at(5);

    // l2
    down = (this->joyMsg.axes.at(2) < -0.5) ? 1 : 0;
    // r2
    up = (this->joyMsg.axes.at(5) < -0.5) ? 1 : 0;
  }

  double y = (left-right) * posStep;
  double z = (up-down) * posStep;

  double yaw = pan * rotStep;
  double pitch = tilt * rotStep;


  // Process last message.
  ignition::math::Vector3d yPos(0.0, y, 0);
  ignition::math::Vector3d zPos(0.0, 0, z);

  ignition::math:: Quaterniond yawRot(0.0, 0.0, yaw);
  ignition::math:: Quaterniond pitchRot(0.0, pitch, 0.0);

  auto myPose = this->model->GetWorldPose().Ign();

  // Y movement in local frame
  ignition::math::Vector3d worldPos = myPose.Pos() +
      myPose.Rot().RotateVector(yPos);

  // Z movement in world frame
  worldPos += zPos;

  // yaw rotation in world frame
  ignition::math::Quaterniond worldQuat = yawRot * myPose.Rot();

  // pitch rotation in local frame
  worldQuat = worldQuat * pitchRot;

  ignition::math::Pose3d newPose(worldPos, worldQuat);
  this->model->SetWorldPose(newPose);
}

/////////////////////////////////////////////////
void FRCBlueCameraPlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  this->joyMsg = *_msg;
}
