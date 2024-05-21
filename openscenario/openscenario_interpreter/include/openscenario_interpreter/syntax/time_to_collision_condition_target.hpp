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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TIME_TO_COLLISION_CONDITION_TARGET_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TIME_TO_COLLISION_CONDITION_TARGET_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   TimeToCollisionConditionTarget 1.3

   Target position used in the TimeToCollisionCondition. Can be defined as
   either an explicit position, or the position of a reference entity.

   <xsd:complexType name="TimeToCollisionConditionTarget">
     <xsd:choice>
       <xsd:element name="Position" type="Position"/>
       <xsd:element name="EntityRef" type="EntityRef"/>
     </xsd:choice>
   </xsd:complexType>
*/
struct TimeToCollisionConditionTarget : public ComplexType
{
  explicit TimeToCollisionConditionTarget(const pugi::xml_node & node, Scope & scope)
  // clang-format off
  : ComplexType(choice(node,
      std::make_pair( "Position", [&](auto && node) { return make< Position>(std::forward<decltype(node)>(node), scope); }),
      std::make_pair("EntityRef", [&](auto && node) { return make<EntityRef>(std::forward<decltype(node)>(node), scope); })))
  // clang-format on
  {
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TIME_TO_COLLISION_CONDITION_TARGET_HPP_
