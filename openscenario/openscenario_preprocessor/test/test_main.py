from time import sleep

import pytest
import os
from openscenario_utility.conversion import convert
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import difflib


@pytest.mark.skip
def test(scenario_name, desired_diff_num):
    # share_dir = get_package_share_directory("openscenario_preprocessor")
    preprocessor_share_dir = get_package_share_directory("openscenario_preprocessor")
    test_runner_share_dir = preprocessor_share_dir + "/../../../scenario_test_runner/share/scenario_test_runner"
    # sample_scenario_path = Path(share_dir + "/test/scenarios/" + scenario_name + ".yaml")
    sample_scenario_path = Path(test_runner_share_dir + "/scenario/" + scenario_name + ".yaml")
    output_dir = Path("/tmp/openscenario_preprocessor/test")
    os.makedirs(output_dir, exist_ok=True)
    convert(sample_scenario_path, output_dir)
    input_xosc = "/tmp/openscenario_preprocessor/test/" + scenario_name + "_0.xosc"

    command_path = preprocessor_share_dir + "/../../lib/openscenario_preprocessor/openscenario_preprocessor_command"
    preprocessor_command = command_path + " -s " + str(sample_scenario_path) + " -o /tmp/openscenario_preprocessor/test" \
                           + " --parameters '{\"random_offset\": true}'" + " -f t4v2" + " --skip-full-derivation"
    print(preprocessor_command)
    os.system(preprocessor_command)
    sleep(1)

    file1 = open(input_xosc)
    file2 = open("/tmp/openscenario_preprocessor/normalized.xosc")
    diff = difflib.Differ()
    output_diff = diff.compare(file1.readlines(), file2.readlines())
    diff_num = 0
    for line in output_diff:
        if line[0:1] in ['+', '-']:
            diff_num += 1
    assert diff_num == desired_diff_num


def test_sample():
    test("sample", 2)


def test_autoware_simple():
    test("autoware-simple", 2)


def test_failure():
    test("failure", 4)