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

#ifndef SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_HPP_
#define SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_HPP_

#include <scenario_simulator/sensor_simulation/sensor_simulation.hpp>
#include <scenario_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <scenario_simulator/sensor_simulation/lidar/lidar_sensor.hpp>

#include <simulation_interface/zmq_server.hpp>
#include <simulation_interface/zmq_multi_server.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <map>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_EXPORT __attribute__((dllexport))
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_EXPORT __declspec(dllexport)
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_BUILDING_DLL
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC \
  SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_EXPORT
#else
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC \
  SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_IMPORT
#endif
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC_TYPE \
  SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_LOCAL
#else
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_LOCAL
#endif
#define SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

namespace scenario_simulator
{
class ScenarioSimulator : public rclcpp::Node
{
public:
  SCENARIO_SIMULATOR_SCENARIO_SIMULATOR_COMPONENT_PUBLIC
  explicit ScenarioSimulator(const rclcpp::NodeOptions & options);
  ~ScenarioSimulator();

private:
  SensorSimulation sensor_sim_;
  void initialize(
    const simulation_api_schema::InitializeRequest & req,
    simulation_api_schema::InitializeResponse & res);
  void updateFrame(
    const simulation_api_schema::UpdateFrameRequest & req,
    simulation_api_schema::UpdateFrameResponse & res);
  void updateEntityStatus(
    const simulation_api_schema::UpdateEntityStatusRequest & req,
    simulation_api_schema::UpdateEntityStatusResponse & res);
  void spawnVehicleEntity(
    const simulation_api_schema::SpawnVehicleEntityRequest & req,
    simulation_api_schema::SpawnVehicleEntityResponse & res);
  void spawnPedestrianEntity(
    const simulation_api_schema::SpawnPedestrianEntityRequest & req,
    simulation_api_schema::SpawnPedestrianEntityResponse & res);
  void despawnEntity(
    const simulation_api_schema::DespawnEntityRequest & req,
    simulation_api_schema::DespawnEntityResponse & res);
  void attachDetectionSensor(
    const simulation_api_schema::AttachDetectionSensorRequest & req,
    simulation_api_schema::AttachDetectionSensorResponse & res);
  void attachLidarSensor(
    const simulation_api_schema::AttachLidarSensorRequest & req,
    simulation_api_schema::AttachLidarSensorResponse & res);
  void updateSensorFrame(
    const simulation_api_schema::UpdateSensorFrameRequest & req,
    simulation_api_schema::UpdateSensorFrameResponse & res);
  std::vector<openscenario_msgs::VehicleParameters> ego_vehicles_;
  std::vector<openscenario_msgs::VehicleParameters> vehicles_;
  std::vector<openscenario_msgs::PedestrianParameters> pedestrians_;
  double realtime_factor_;
  double step_time_;
  double current_time_;
  bool initialized_;
  std::vector<openscenario_msgs::EntityStatus> entity_status_;
  zeromq::MultiServer server_;
};
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_HPP_
