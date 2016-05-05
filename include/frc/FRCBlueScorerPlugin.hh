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

#ifndef _GAZEBO_FRC_BLUE_SCORER_PLUGIN_HH_
#define _GAZEBO_FRC_BLUE_SCORER_PLUGIN_HH_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#pragma GCC diagnostic pop
#include <sdf/sdf.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{

  class GAZEBO_VISIBLE InternalJointController
  {
    /// \brief ToDo.
    public: InternalJointController(const physics::JointPtr _joint,
      const double _p, const double _d,
      const double _lowerTarget, const double _upperTarget,
      const int _buttonIndex);

    /// \brief ToDo.
    public: void Update(const sensor_msgs::Joy &_joyMsg);

    /// \brief ToDo.
    private: void Toggle();

    /// \brief ToDo.
    private: physics::JointPtr joint;
    /// \brief ToDo.
    private: double lowerTarget;
    /// \brief ToDo.
    private: double upperTarget;
    /// \brief ToDo.
    private: gazebo::common::PID pid;

    private: double target;

    private: int buttonIndex;

    private: gazebo::common::Time lastToggleTime;

    /// \brief Time of the last update.
    private: gazebo::common::Time lastControllerUpdate;
  };

  /// \class FRCBlueScorerPlugin FRCBlueScorerPlugin.hh
  /// \brief ToDo.
  class GAZEBO_VISIBLE FRCBlueScorerPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: FRCBlueScorerPlugin() = default;

    /// \brief Destructor.
    public: ~FRCBlueScorerPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the foosball table rods.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief ToDo.
    void OnData(const sensor_msgs::Joy::ConstPtr& _msg);

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    /// \brief Joystick topic.
    private: std::string topic;

    /// \brief ToDo.
    private: ros::NodeHandle *n;

    /// \brief ToDo.
    private: ros::Subscriber sub;

    /// \brief ToDo.
    private: sensor_msgs::Joy joyMsg;

    private: std::vector<InternalJointController*> controllers;
  };

}
#endif
