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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/speed_action_target.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/transition_dynamics.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedAction ------------------------------------------------------------
 *
 *  <xsd:complexType name="SpeedAction">
 *    <xsd:all>
 *      <xsd:element name="SpeedActionDynamics" type="TransitionDynamics"/>
 *      <xsd:element name="SpeedActionTarget" type="SpeedActionTarget"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SpeedAction
{
  Scope inner_scope;

  const TransitionDynamics speed_action_dynamics;

  const SpeedActionTarget speed_action_target;

  template <typename Node>
  explicit SpeedAction(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope),
    speed_action_dynamics(
      readElement<TransitionDynamics>("SpeedActionDynamics", node, inner_scope)),
    speed_action_target(readElement<SpeedActionTarget>("SpeedActionTarget", node, inner_scope))
  {
  }

  std::unordered_map<String, Boolean> accomplishments;

  auto reset()
  {
    accomplishments.clear();
    for (const auto & actor : inner_scope.actors) {
      accomplishments.emplace(actor, false);
    }
  }

  decltype(auto) operator()(const Scope::Actor & actor)
  {
    //<<<<<<< HEAD
    std::function<double()> calc_absolute_target_speed;
    std::function<bool(const Scope::Actor &)> is_end;
    std::tie(calc_absolute_target_speed, is_end) = speed_action_target();

    double current_absolute_target_speed = calc_absolute_target_speed();
    switch (speed_action_dynamics.dynamics_shape) {
      case DynamicsShape::step: {
        auto status = getEntityStatus(actor);
        status.action_status.twist.linear.x = current_absolute_target_speed;
        setEntityStatus(actor, status);
        setTargetSpeed(actor, current_absolute_target_speed, true);
        break;
      }
      case DynamicsShape::linear: {
        setTargetSpeed(actor, current_absolute_target_speed, true);
        break;
      }
      default:
        throw UNSUPPORTED_SETTING_DETECTED(SpeedAction, speed_action_dynamics.dynamics_shape);
    }

    if (speed_action_target.is<RelativeTargetSpeed>()) {
      // dynamics_shape is not taken in account
      check = [calc_absolute_target_speed, is_end = std::move(is_end)](const Scope::Actor & actor) {
        setTargetSpeed(actor, calc_absolute_target_speed(), true);
        return is_end(actor);
      };
    } else {
      check = std::move(is_end);
    }
  }

  auto start()
  {
    reset();

    for (const auto & actor : inner_scope.actors) {
      (*this)(actor);
    }

    return unspecified;
  }

  auto update()
  {
    for (auto && each : accomplishments) {
      each.second = each.second or (check ? check(Scope::Actor(each.first)) : true);
    }
  }

  auto accomplished()
  {
    update();
    return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
  }

private:
  std::function<bool(const Scope::Actor &)> check;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_
