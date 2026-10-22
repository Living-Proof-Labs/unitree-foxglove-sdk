"""Create reviewable JUnit reports for unit and simulation Task results."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

SUITES = ("unit", "e2e-fall", "e2e", "e2e-hallway")


def log_tail(path: Path) -> str:
    if not path.exists():
        return "No command log was recorded."
    text = path.read_text(errors="replace")[-8000:]
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", text)
    return re.sub(r"[\x00-\x08\x0b\x0c\x0e-\x1f\ud800-\udfff\ufffe\uffff]", "", text)


def stage_case(directory: Path, name: str, source: str, *, skipped: bool = False) -> ET.Element:
    case = ET.Element("testcase", name=name, classname=directory.name,
                      file=source, line="1", time="0")
    metadata = directory / "run.json"
    if metadata.exists():
        run = json.loads(metadata.read_text())
        case.set("time", str(run["seconds"]))
        if run["exit_code"] != 0:
            outcome = "failure" if name == "acceptance" else "error"
            message = f"{' '.join(run['command'])}: exit {run['exit_code']}"
            ET.SubElement(case, outcome, message=message).text = log_tail(directory / "task.log")
    elif skipped:
        ET.SubElement(case, "skipped", message="Not started because an earlier stage failed")
    else:
        ET.SubElement(case, "error", message="No result was recorded").text = log_tail(
            directory / "task.log"
        )
    return case


def write_suite(path: Path, name: str, cases: list[ET.Element]) -> None:
    suite = ET.Element("testsuite", name=name, tests=str(len(cases)),
                       failures=str(sum(case.find("failure") is not None for case in cases)),
                       errors=str(sum(case.find("error") is not None for case in cases)),
                       skipped=str(sum(case.find("skipped") is not None for case in cases)),
                       time=str(sum(float(case.get("time", "0")) for case in cases)))
    suite.extend(cases)
    root = ET.Element("testsuites")
    root.append(suite)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def create_report(directory: Path, suite: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    output = directory / "test-results.xml"
    junit = directory / "junit.xml"
    if suite == "unit" and junit.exists():
        # Preserve pytest's individual cases, durations, failure details, and file paths.
        ET.parse(junit)
        shutil.copyfile(junit, output)
        return output
    if suite == "unit":
        case = ET.Element("testcase", classname="unit", name="unit_suite",
                          file="Taskfile.yml", line="1", time="0")
        ET.SubElement(case, "error", message="pytest did not produce a JUnit report").text = (
            log_tail(directory / "task.log")
        )
        write_suite(output, suite, [case])
        return output

    source = "scripts/e2e_hallway.sh" if suite == "e2e-hallway" else "scripts/e2e.sh"
    build = stage_case(directory / "build", "image_build", "Dockerfile")
    acceptance = stage_case(directory, "acceptance", source,
                            skipped=build.find("error") is not None)
    replay = stage_case(directory / "render", "replay", "src/unitree_foxglove/render_mcap.py",
                        skipped=not (directory / "run.json").exists())
    # Stable class names prevent build/render cases from colliding across scenarios.
    for case in (build, acceptance, replay):
        case.set("classname", f"simulation.{suite}")
    write_suite(output, suite, [build, acceptance, replay])
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=SUITES)
    args = parser.parse_args()
    path = create_report(Path("artifacts") / args.suite, args.suite)
    root = ET.parse(path).getroot()
    cases = list(root.iter("testcase"))
    counts = {outcome: sum(case.find(outcome) is not None for case in cases)
              for outcome in ("failure", "error", "skipped")}
    passed = len(cases) - sum(counts.values())
    print(f"Wrote {path}: {passed} passed, {counts['failure']} failed, "
          f"{counts['error']} errors, {counts['skipped']} skipped")


if __name__ == "__main__":
    main()
