// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/syntax/story.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto Story::accomplished() const -> bool
{
  // NOTE: A Story's goal is accomplished when all its Acts are in the completeState.
  return std::all_of(std::begin(*this), std::end(*this), [](auto && each) {
    return each.template as<Act>().complete();
  });
}

auto Story::ready() noexcept -> bool { return true; }

auto Story::run() -> void
{
  for (auto && act : *this) {
    act.evaluate();
  }
}

auto Story::start() noexcept -> void {}

auto Story::stop() -> void
{
  for (auto && each : *this) {
    each.as<Act>().override();
    each.evaluate();
  }
}

auto Story::stopTriggered() noexcept -> bool { return false; }

auto operator<<(nlohmann::json & json, const Story & datum) -> nlohmann::json &
{
  json["name"] = datum.name;

  json["currentState"] = boost::lexical_cast<std::string>(datum.currentState());

  json["Act"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json act;
    act << each.as<Act>();
    json["Act"].push_back(act);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
