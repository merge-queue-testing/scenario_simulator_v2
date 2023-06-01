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

#include <boost/program_options.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <nlohmann/json.hpp>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>
#include <openscenario_preprocessor/t4v2.hpp>
#include <openscenario_validator/schema.hpp>

const std::string_view template_scenario = R"###(
<OpenSCENARIO>
    <FileHeader author="" date="2022-03-04T18:06:53+09:00" description="" revMajor="0" revMinor="0"/>
    <ParameterValueDistribution>
        <ScenarioFile filepath=""/>
        <Deterministic>
            <DeterministicMultiParameterDistribution>
                <ValueSetDistribution>
                    <ParameterValueSet>
                    </ParameterValueSet>
                </ValueSetDistribution>
            </DeterministicMultiParameterDistribution>
        </Deterministic>
    </ParameterValueDistribution>
</OpenSCENARIO>)###";

auto create_parameter_value_distribution_from_json(
  const boost::filesystem::path & scenario_path, const nlohmann::json & json_parameters)
{
  pugi::xml_document script;
  script.load_string(template_scenario.data());

  script.document_element()
    .select_node(pugi::xpath_query{"/OpenSCENARIO/ParameterValueDistribution/ScenarioFile"})
    .node()
    .attribute("filepath")
    .set_value(scenario_path.c_str());

  auto value_set_node =
    script.document_element()
      .select_node(pugi::xpath_query{
        "/OpenSCENARIO/ParameterValueDistribution/Deterministic/"
        "DeterministicMultiParameterDistribution/ValueSetDistribution/ParameterValueSet"})
      .node();

  for (auto const & json_parameter : json_parameters.items()) {
    auto parameter_assignment_node = value_set_node.append_child("ParameterAssignment");
    parameter_assignment_node.append_attribute("parameterRef") = json_parameter.key().c_str();
    parameter_assignment_node.append_attribute("value") =
      json_parameter.value().get<std::string>().c_str();
  }
  return script;
}

int main(const int argc, char const * const * const argv)
try {
  using namespace boost::program_options;

  options_description description("openscenario_preprocessor_command");

  //  cspell: ignore multitoken
  description.add_options()(
    "output-directory,o",
    value<std::string>()->default_value("/tmp/openscenario_preprocessor/derived"),
    "path of output directory")(
    "format,f", value<openscenario_preprocessor::ScenarioFormat>()->multitoken(),
    "output scenario format (t4v2 / xosc)")(
    "parameters,p", value<std::string>()->default_value("null"), "parameters in json format")(
    "scenario,s", value<std::string>(), "path of scenario file")("skip-full-derivation", "")(
    "help,H", "help");

  variables_map vm;
  store(parse_command_line(argc, argv, description), vm);
  notify(vm);

  auto output_directory_option = boost::filesystem::path(vm["output-directory"].as<std::string>());
  auto format_option = vm["format"].as<openscenario_preprocessor::ScenarioFormat>();
  auto parameters_option = boost::filesystem::path(vm["parameters"].as<std::string>());
  auto scenario_option = boost::filesystem::path(vm["scenario"].as<std::string>());
  bool skip_full_derivation_option = (vm.count("skip-full-derivation") > 0);

  auto scenario_path = boost::filesystem::path(scenario_option);

  boost::filesystem::path tmp_output_directory = "/tmp/openscenario_preprocessor";
  if (not boost::filesystem::exists(tmp_output_directory)) {
    boost::filesystem::create_directories(tmp_output_directory);
  }

  std::vector<boost::filesystem::path> xosc_scenario_paths;

  boost::filesystem::path scenario_modifiers_path{};

  // setup xsd file
  auto file = std::ofstream("/tmp/openscenario_preprocessor/schema.xsd", std::ios::trunc);
  file << openscenario_validator::schema;
  file.close();

  // preprocess t4v2 format and convert to xosc scenarios
  if (scenario_path.extension() == ".yaml" or scenario_path.extension() == ".yml") {
    openscenario_preprocessor::T4V2 t4v2;

    auto [modifiers_path, base_scenario_path] = t4v2.splitScenarioModifiers(scenario_path);
    scenario_modifiers_path = modifiers_path;

    auto xosc_string_scenarios = [&]() {
      if (skip_full_derivation_option) {
        return t4v2.deriveToXoscStringScenarios(base_scenario_path);
      } else {
        return t4v2.deriveToXoscStringScenarios(base_scenario_path, modifiers_path);
      }
    }();

    boost::filesystem::path t4v2_output_directory = "/tmp/openscenario_preprocessor/t4v2_derived";
    if (not boost::filesystem::exists(t4v2_output_directory)) {
      boost::filesystem::create_directories(t4v2_output_directory);
    }
    for (const auto & xosc_string_scenario : xosc_string_scenarios | boost::adaptors::indexed()) {
      boost::filesystem::path xosc_scenario_path =
        t4v2_output_directory / (std::to_string(xosc_string_scenario.index()) + ".xosc");
      std::ofstream ofs(xosc_scenario_path.c_str());
      ofs << xosc_string_scenario.value().c_str();
      ofs.close();
      xosc_scenario_paths.push_back(xosc_scenario_path);
    }
  } else {
    xosc_scenario_paths.push_back(scenario_path);
  }

  // derive for given parameters
  if (parameters_option != "null") {
    for (auto scenario_path : xosc_scenario_paths) {
      auto parameters_json = nlohmann::json::parse(parameters_option.c_str());
      // convert value to string, because current implementation is not support other types
      for (auto & json_item : parameters_json.items()) {
        if (not json_item.value().is_string()) {
          try {
            json_item.value() = boost::lexical_cast<std::string>(json_item.value());
          } catch (std::exception & e) {
            std::stringstream what;
            what << "Cannot convert parameter value, type  " << json_item.value().type_name()
                 << " to string  : " << e.what();
            throw std::runtime_error(what.str());
          }
        }
      }

      auto parameter_value_distribution =
        create_parameter_value_distribution_from_json(scenario_path, parameters_json);

      parameter_value_distribution.save_file(
        "/tmp/openscenario_preprocessor/parameter_value_distribution.xosc");

      openscenario_preprocessor::Preprocessor preprocessor(output_directory_option);

      preprocessor.preprocessScenario(
        "/tmp/openscenario_preprocessor/parameter_value_distribution.xosc", format_option);

      // merge scenario modifiers if skip_full_derivation_option is ON
      if (
        format_option == openscenario_preprocessor::ScenarioFormat::t4v2 &&
        skip_full_derivation_option) {
        auto derived_scenario_paths = preprocessor.getPreprocessedScenarios();
        while (not derived_scenario_paths.empty()) {
          std::ifstream input_scenario_file{derived_scenario_paths.front().string()};
          std::stringstream scenario_ss;
          scenario_ss << input_scenario_file.rdbuf();
          input_scenario_file.close();

          std::ofstream scenario_file{derived_scenario_paths.front().string()};
          std::ifstream modifiers_file{scenario_modifiers_path.string()};
          scenario_file << modifiers_file.rdbuf();
          modifiers_file.close();

          scenario_file << scenario_ss.str();
          scenario_file.close();

          derived_scenario_paths.pop();
        }
      }
    }
  } else {
    throw std::runtime_error("parameters option is required");
  }
  return 0;
} catch (std::exception & e) {
  std::cerr << "Caught an exception : " << e.what() << std::endl;
  return 1;
}