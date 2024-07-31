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

#include <geometry/vector3/norm.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SpeedCondition::SpeedCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: Scope(scope),
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  direction(readAttribute<DirectionalDimension>("direction", node, scope, std::nullopt)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), {Double::nan()})
{
}

auto SpeedCondition::description() const -> String
{
  std::stringstream description;

  description << triggering_entities.description() << "'s speed = ";

  print_to(description, results);

  description << " " << rule << " " << value << "?";

  return description.str();
}

auto SpeedCondition::evaluate(
  const Entities * entities, const Entity & triggering_entity,
  const std::optional<DirectionalDimension> & direction) -> double
{
  if (entities->isAdded(triggering_entity)) {
    if (const auto v = evaluateSpeed(triggering_entity); direction) {
      switch (*direction) {
        case DirectionalDimension::longitudinal:
          return v.x;
        case DirectionalDimension::lateral:
          return v.y;
        case DirectionalDimension::vertical:
          return v.z;
        default:
          return math::geometry::norm(v);
      }
    } else {
      return math::geometry::norm(v);
    }
  } else {
    return Double::nan();
  }
}

auto SpeedCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
    results.push_back(triggering_entity.apply([&](const auto & triggering_entity) {
      return evaluate(global().entities, triggering_entity);
    }));
    return not results.back().size() or std::invoke(rule, results.back(), value).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
