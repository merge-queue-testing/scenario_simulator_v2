// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef SIMULATION_INTERFACE__CONVERSIONS_HPP_
#define SIMULATION_INTERFACE__CONVERSIONS_HPP_

#include <geometry_msgs.pb.h>
#include <openscenario_msgs.pb.h>
#include <simulation_api_schema.pb.h>

#include <exception>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <openscenario_msgs/msg/action_status.hpp>
#include <openscenario_msgs/msg/axle.hpp>
#include <openscenario_msgs/msg/axles.hpp>
#include <openscenario_msgs/msg/bounding_box.hpp>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <openscenario_msgs/msg/lanelet_pose.hpp>
#include <openscenario_msgs/msg/pedestrian_parameters.hpp>
#include <openscenario_msgs/msg/performance.hpp>
#include <openscenario_msgs/msg/property.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <simulation_interface/constants.hpp>
#include <string>
#include <vector>

namespace simulation_interface
{
class XmlParameterError : public std::runtime_error
{
public:
  explicit XmlParameterError(std::string message, const char * file, int line)
  : runtime_error(message + "\nFile:" + file + "\nLine:" + std::to_string(line))
  {
  }
};

#define THROW_XML_PARAMETER_ERROR(description) \
  throw XmlParameterError(description, __FILE__, __LINE__);

#define THROW_XML_PARAMETER_NOT_DEFINED_ERROR(name) \
  throw XmlParameterError(                          \
    std::string("parameter : ") + name + std::string(" does not defined."), __FILE__, __LINE__);

class ProtobufConversionError : public std::runtime_error
{
public:
  explicit ProtobufConversionError(std::string message, const char * file, int line)
  : runtime_error(message + "\nFile:" + file + "\nLine:" + std::to_string(line))
  {
  }
};

#define THROW_PROTOBUF_PARAMETER_ERROR(description) \
  throw ProtobufConversionError(description, __FILE__, __LINE__);

void toProto(const geometry_msgs::msg::Point & p, geometry_msgs::Point & proto);
void toMsg(const geometry_msgs::Point & proto, geometry_msgs::msg::Point & p);
void toProto(const geometry_msgs::msg::Quaternion & q, geometry_msgs::Quaternion & proto);
void toMsg(const geometry_msgs::Quaternion & proto, geometry_msgs::msg::Quaternion & q);
void toProto(const geometry_msgs::msg::Pose & p, geometry_msgs::Pose & proto);
void toMsg(const geometry_msgs::Pose & proto, geometry_msgs::msg::Pose & p);
void toProto(const geometry_msgs::msg::Vector3 & v, geometry_msgs::Vector3 & proto);
void toMsg(const geometry_msgs::Vector3 & proto, geometry_msgs::msg::Vector3 & v);
void toProto(const geometry_msgs::msg::Twist & t, geometry_msgs::Twist & proto);
void toMsg(const geometry_msgs::Twist & proto, geometry_msgs::msg::Twist & t);
void toProto(const geometry_msgs::msg::Accel & a, geometry_msgs::Accel & proto);
void toMsg(const geometry_msgs::Accel & proto, geometry_msgs::msg::Accel & a);
void toProto(
  const openscenario_msgs::msg::BoundingBox & box, openscenario_msgs::BoundingBox & proto);
void toMsg(const openscenario_msgs::BoundingBox & proto, openscenario_msgs::msg::BoundingBox & box);
void toProto(
  const openscenario_msgs::msg::Performance & performance, openscenario_msgs::Performance & proto);
void toMsg(
  const openscenario_msgs::Performance & proto, openscenario_msgs::msg::Performance & performance);
void toProto(const openscenario_msgs::msg::Axle & axle, openscenario_msgs::Axle & proto);
void toMsg(const openscenario_msgs::Axle & proto, openscenario_msgs::msg::Axle & axle);
void toProto(const openscenario_msgs::msg::Axles & axles, openscenario_msgs::Axles & proto);
void toMsg(const openscenario_msgs::Axles & proto, openscenario_msgs::msg::Axles & axles);
/*
void toProto(
  const openscenario_msgs::msg::Property & p,
  openscenario_msgs::Property & proto);
void toMsg(
  const openscenario_msgs::Property & proto,
  openscenario_msgs::msg::Property & p);
*/
void toProto(
  const openscenario_msgs::msg::VehicleParameters & p,
  openscenario_msgs::VehicleParameters & proto);
void toMsg(
  const openscenario_msgs::VehicleParameters & proto,
  openscenario_msgs::msg::VehicleParameters & p);
void toProto(
  const openscenario_msgs::msg::PedestrianParameters & p,
  openscenario_msgs::PedestrianParameters & proto);
void toMsg(
  const openscenario_msgs::PedestrianParameters & proto,
  openscenario_msgs::msg::PedestrianParameters & p);
void toProto(
  const openscenario_msgs::msg::ActionStatus & s, openscenario_msgs::ActionStatus & proto);
void toMsg(openscenario_msgs::ActionStatus & proto, const openscenario_msgs::msg::ActionStatus & s);
void toProto(
  const openscenario_msgs::msg::LaneletPose & pose, openscenario_msgs::LaneletPose & proto);
void toMsg(
  const openscenario_msgs::LaneletPose & proto, openscenario_msgs::msg::LaneletPose & pose);
void toProto(
  const openscenario_msgs::msg::EntityType & type, openscenario_msgs::EntityType & proto);
void toMsg(const openscenario_msgs::EntityType & proto, openscenario_msgs::msg::EntityType & type);
void toProto(
  const openscenario_msgs::msg::EntityStatus & status, openscenario_msgs::EntityStatus & proto);
void toMsg(
  const openscenario_msgs::EntityStatus & proto, openscenario_msgs::msg::EntityStatus & status);
}  // namespace simulation_interface

#endif  // SIMULATION_INTERFACE__CONVERSIONS_HPP_