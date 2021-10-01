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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORY_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/act.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Story ------------------------------------------------------------------
 *
 *  <xsd:complexType name="Story">
 *    <xsd:sequence>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="Act" maxOccurs="unbounded" type="Act"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Story : public Scope, public StoryboardElement<Story>, public Elements
{
  template <typename Node>
  explicit Story(const Node & node, Scope & scope)
  : Scope(scope.makeChildScope(readAttribute<String>("name", node, scope)))
  {
    callWithElements(node, "ParameterDeclarations", 0, 1, [&](auto && node) {
      return make<ParameterDeclarations>(node, localScope());
    });

    callWithElements(node, "Act", 1, unbounded, [&](auto && node) {
      return push_back(readStoryboardElement<Act>(node, localScope()));
    });
  }

  using StoryboardElement::evaluate;

  /*  */ auto accomplished() const -> bool;

  static auto ready() noexcept -> bool;

  /*  */ auto run() -> void;

  static auto start() noexcept -> void;

  /*  */ auto stop() -> void;

  static auto stopTriggered() noexcept -> bool;
};

auto operator<<(nlohmann::json &, const Story &) -> nlohmann::json &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORY_HPP_
