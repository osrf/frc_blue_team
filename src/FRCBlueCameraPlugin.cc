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

/*  this->node.reset();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->factoryPub =
    this->node->Advertise<msgs::Factory>("~/factory");
  this->factoryPub->WaitForConnection();*/
}

/////////////////////////////////////////////////
void FRCBlueCameraPlugin::Update(const common::UpdateInfo & /*_info*/)
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
void FRCBlueCameraPlugin::OnData(const sensor_msgs::Joy::ConstPtr& _msg)
{
  std::cout << "Joy received" << std::endl;
  this->joyMsg = *_msg;
}

/*/////////////////////////////////////////////////
void FRCBlueCameraPlugin::SpawnCamera(const std::string &_name)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  std::string modelName = _name + "_model";
  std::string cameraName = _name + "_sensor";
  ignition::math::Vector3d pos(0, 0, 1.8);
  ignition::math::Vector3d rpy(0, 0, 0);
  double rate = 30;
  int width = 1080;
  int height = 720;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << modelName << "'>"
    << "<static>true</static>"
    << "<pose>" << pos << " " << rpy << "</pose>"
    << "<link name ='body'>"
    << "  <sensor name ='" << cameraName
    << "' type ='camera'>"
    << "    <always_on>1</always_on>"
    << "    <update_rate>" << rate << "</update_rate>"
    << "    <visualize>true</visualize>"
    << "    <camera>"
    << "      <horizontal_fov>0.78539816339744828</horizontal_fov>"
    << "      <image>"
    << "        <width>" << width << "</width>"
    << "        <height>" << height << "</height>"
    << "        <format>R8G8B8</format>"
    << "      </image>"
    << "      <clip>"
    << "        <near>0.1</near><far>100</far>"
    << "      </clip>"
    << "    </camera>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);
}*/
