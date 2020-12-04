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

#include <openscenario_interpreter/procedure.hpp>

#include <memory>

namespace openscenario_interpreter
{
static typename std::aligned_storage<
  sizeof(scenario_simulator::API),
  alignof(scenario_simulator::API)
>::type memory;

scenario_simulator::API & connection {
  reinterpret_cast<scenario_simulator::API &>(memory)
};

// static int schwarz_counter { 0 };
//
// Connector::Connector()
// {
//   if (schwarz_counter++ == 0)
//   {
//     new (&connection) scenario_simulator::API();
//   }
// }
//
// Connector::~Connector()
// {
//   if (--schwarz_counter == 0)
//   {
//     connection.~API();
//   }
// }

}  // namespace openscenario_interpreter