"""Result publishing must preserve failures and distinguish missing results from success."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

from unitree_foxglove.test_report import create_report


def result(path: Path, code: int) -> None:
    path.mkdir(parents=True, exist_ok=True)
    (path / "run.json").write_text(json.dumps({
        "command": ["task", "check"], "exit_code": code, "seconds": 2.5,
    }))
    (path / "task.log").write_text("\x1b[31mRobot fell\x1b[0m\x00\n")


def test_unit_report_preserves_individual_cases_and_failure_details(tmp_path: Path) -> None:
    original = '<testsuites><testsuite><testcase name="pose" classname="robot" time="0.5">'
    original += '<failure message="wrong pose">expected upright</failure></testcase>'
    original += '<testcase name="scan"><skipped/></testcase></testsuite></testsuites>'
    (tmp_path / "junit.xml").write_text(original)
    output = create_report(tmp_path, "unit")
    assert output.read_text() == original
    cases = list(ET.parse(output).getroot().iter("testcase"))
    assert cases[0].find("failure").text == "expected upright"
    assert cases[1].find("skipped") is not None


def test_missing_unit_report_is_an_error_not_an_empty_pass(tmp_path: Path) -> None:
    output = create_report(tmp_path, "unit")
    suite = ET.parse(output).getroot().find("testsuite")
    assert suite.get("errors") == "1"
    assert suite.find("testcase/error") is not None


def test_simulation_failure_preserves_successful_replay_and_sanitizes_logs(tmp_path: Path) -> None:
    result(tmp_path / "build", 0)
    result(tmp_path, 201)
    result(tmp_path / "render", 0)
    output = create_report(tmp_path, "e2e-hallway")
    suite = ET.parse(output).getroot().find("testsuite")
    assert suite.get("tests") == "3"
    assert suite.get("failures") == "1"
    assert suite.get("errors") == "0"
    assert suite.get("time") == "7.5"
    cases = suite.findall("testcase")
    assert cases[1].find("failure").text == "Robot fell\n"
    assert cases[2].find("failure") is None and cases[2].find("error") is None
    assert cases[1].get("file") == "scripts/e2e_hallway.sh"
    assert {case.get("classname") for case in cases} == {"simulation.e2e-hallway"}


def test_build_failure_skips_stages_that_did_not_start(tmp_path: Path) -> None:
    result(tmp_path / "build", 1)
    suite = ET.parse(create_report(tmp_path, "e2e-fall")).getroot().find("testsuite")
    assert suite.get("errors") == "1"
    assert suite.get("skipped") == "2"


def test_missing_simulation_results_is_an_error(tmp_path: Path) -> None:
    suite = ET.parse(create_report(tmp_path, "e2e")).getroot().find("testsuite")
    assert suite.get("errors") == "1"
    assert suite.get("skipped") == "2"


def test_replay_failure_is_a_separate_tooling_error(tmp_path: Path) -> None:
    result(tmp_path / "build", 0)
    result(tmp_path, 0)
    result(tmp_path / "render", 1)
    suite = ET.parse(create_report(tmp_path, "e2e-fall")).getroot().find("testsuite")
    assert suite.get("failures") == "0"
    assert suite.get("errors") == "1"
    assert suite.find("testcase[@name='replay']/error") is not None
