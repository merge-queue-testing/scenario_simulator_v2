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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/stochastic.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/**
 * Note: Stochastic.randomSeed is initialized with 0, if it is not specified in scenario.
 *       The behavior related to this implementation is not specified in the OpenSCENARIO standard,
 *       so it may change in the future.
 */
Stochastic::Stochastic(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  number_of_test_runs(readAttribute<UnsignedInt>("numberOfTestRuns", node, scope)),
  random_seed([&] {
    auto seed = static_cast<double>(readAttribute<Double>("randomSeed", node, scope, 0));
    scope.random_engine.seed(seed);
    return seed;
  }()),
  stochastic_distributions(
    readElements<StochasticDistribution, 1>("StochasticDistribution", node, scope))
{
}

auto Stochastic::derive() -> ParameterDistribution
{
  ParameterDistribution distribution;
  for (std::size_t i = 0; i < number_of_test_runs; i++) {
    ParameterListSharedPtr parameter_list = std::make_shared<ParameterList>();
    for (auto & stochastic_distribution : stochastic_distributions) {
      auto derived = stochastic_distribution.derive();
      parameter_list->emplace(stochastic_distribution.parameter_name, derived);
    }
    distribution.emplace_back(parameter_list);
  }
  return distribution;
}

auto Stochastic::derive(
  std::size_t local_index, std::size_t local_size, std::size_t global_index, std::size_t global_size) -> ParameterList
{
  // update random_engine
  random_engine.seed(random_seed);
  random_engine.discard(global_index);

  // N test_runs : i (0 <= i < N)
  // M distributions : j (0 <= j < M)
  // index : i * M + j
  return std::next(stochastic_distributions.begin(), local_index % stochastic_distributions.size())
    ->derive(local_index, local_size, global_index, global_size);
}

}  // namespace syntax
}  // namespace openscenario_interpreter
