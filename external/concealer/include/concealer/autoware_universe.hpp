// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONCEALER__AUTOWARE_UNIVERSE_HPP_
#define CONCEALER__AUTOWARE_UNIVERSE_HPP_

#include <concealer/autoware.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <concealer/utility/subscriber_wrapper.hpp>
#include <concealer/utility/publisher_wrapper.hpp>

namespace concealer {


class AutowareUniverse : public Autoware
{
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;
  SubscriberWrapper<AckermannControlCommand> getAckermannControlCommand;
  SubscriberWrapper<GearCommand> getGearCommandImpl;

  using Acceleration = geometry_msgs::msg::AccelWithCovarianceStamped;
  using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  using ControlModeReport = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  PublisherWrapper<Acceleration> setAcceleration;
  PublisherWrapper<SteeringReport> setSteeringReport;
  PublisherWrapper<GearReport> setGearReport;
  PublisherWrapper<ControlModeReport> setControlModeReport;

public:

  CONCEALER_PUBLIC explicit AutowareUniverse()
  : getAckermannControlCommand("/control/command/control_cmd", *this)
  , getGearCommandImpl("/control/command/gear_cmd", *this)
  , setAcceleration("/localization/acceleration", *this)
  , setSteeringReport("/vehicle/status/steering_status", *this)
  , setGearReport("/vehicle/status/gear_status", *this)
  , setControlModeReport("/vehicle/status/control_mode", *this)
  {}

  auto getAcceleration() const -> double override;

  auto getSteeringAngle() const -> double override;

  auto getVelocity() const -> double override;

  auto update() -> void override;

  auto getGearCommand() const -> autoware_auto_vehicle_msgs::msg::GearCommand override;

  auto getGearSign() const -> double override;

  auto getVehicleCommand() const -> std::tuple<
      autoware_auto_control_msgs::msg::AckermannControlCommand,
      autoware_auto_vehicle_msgs::msg::GearCommand> override;
};

}

#endif //CONCEALER__AUTOWARE_UNIVERSE_HPP_
