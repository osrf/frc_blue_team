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

#ifndef _GAZEBO_FRC_BLUE_ROBOT_PLUGIN_HH_
#define _GAZEBO_FRC_BLUE_ROBOT_PLUGIN_HH_

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
  /// \class FRCBlueAckermannDrivePlugin FRCBlueAckermannDrivePlugin.hh
  /// \brief ToDo.
  class GAZEBO_VISIBLE FRCBlueAckermannDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: FRCBlueAckermannDrivePlugin() = default;

    /// \brief Destructor.
    public: ~FRCBlueAckermannDrivePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Update the foosball table rods.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief ToDo.
    void OnData(const sensor_msgs::Joy::ConstPtr& _msg);

    /// \brief Find revolute joints in the model.
    private: void FindJoints();

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

    /// \brief Revolute joint for moving the left wheel of the vehicle.
    private: physics::JointPtr leftJoint;

    /// \brief Revolute joint for moving the right wheel of the vehicle.
    private: physics::JointPtr rightJoint;


    /// \brief Revolute joint for moving the front left wheel of the vehicle.
    private: physics::JointPtr frontLeftJoint;

    /// \brief Revolute joint for moving the front right wheel of the vehicle.
    private: physics::JointPtr frontRightJoint;

    /// \brief Revolute joints for steering the front wheels of the vehicle.
    private: physics::JointPtr steeringJoints[2];

    /// \brief Left/Right wheel speed.
    private: double wheelSpeed[2];

    /// \brief Wheel separation.
    private: double wheelSeparation;

    /// \brief Wheel radius.
    private: double wheelRadius;

    /// \brief Max torque to apply to wheel joints
    private: double torque;

    /// \brief Max torque to apply to steering joints
    private: double turnTorque;

    /// \brief Whether or not to flip the steering controls
    private: bool flipSteering;

    /// \brief Time of the last button toggle.
    private: gazebo::common::Time lastToggleTime;
  };

}
#endif
