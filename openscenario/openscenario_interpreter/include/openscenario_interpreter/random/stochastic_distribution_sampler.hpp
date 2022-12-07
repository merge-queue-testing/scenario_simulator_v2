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

#ifndef OPENSCENARIO_INTERPRETER__STOCHASTIC_DISTRIBUTION_SAMPLER_HPP_
#define OPENSCENARIO_INTERPRETER__STOCHASTIC_DISTRIBUTION_SAMPLER_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <random>

namespace openscenario_interpreter
{
inline namespace random
{
template <typename DistributorT>
struct StochasticDistributionSampler
{
  template <typename... Ts>
  explicit StochasticDistributionSampler(Ts &&... xs)
  : distribute(std::forward<decltype(xs)>(xs)...)
  {
  }

  DistributorT distribute;

  auto operator()(std::mt19937 & random_engine) { return distribute(random_engine); }
};

struct SingleParameterList : public std::vector<Object> {};
//using UnnamedParameterSet = std::vector<Object>;
//using UnnamedParameterList = std::vector<UnnamedParameterSet>;

//using ParameterSet = std::unordered_map<std::string,Object>;
//using ParameterList = std::vector<ParameterSet>;

}  // namespace random
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__STOCHASTIC_DISTRIBUTION_SAMPLER_HPP_
