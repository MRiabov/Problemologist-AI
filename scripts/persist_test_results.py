import sys
import os
import json
import xml.etree.ElementTree as ET
from datetime import datetime
from pydantic import BaseModel, Field
from typing import List, Optional


# Define models
class TestCaseResult(BaseModel):
    name: str
    classname: str
    time: float
    status: str
    message: Optional[str] = None


class TestRun(BaseModel):
    timestamp: str
    status: str
    total: int
    passed: int
    failed: int
    skipped: int
    errors: int
    duration: float
    tests: List[TestCaseResult]


class TestHistory(BaseModel):
    runs: List[TestRun]


HISTORY_FILE = "test_output/integration_test_history.json"
ARCHIVE_FILE = "test_output/integration_test_archive.json"


def parse_junit(xml_path):
    if not os.path.exists(xml_path):
        print(f"Junit XML not found at {xml_path}")
        return None

    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()

        # In pytest, root is usually 'testsuites'
        if root.tag == "testsuites":
            testsuite = root.find("testsuite")
        else:
            testsuite = root

        if testsuite is None:
            print("No testsuite found in XML")
            return None

        total = int(testsuite.get("tests", 0))
        failures = int(testsuite.get("failures", 0))
        errors = int(testsuite.get("errors", 0))
        skipped = int(testsuite.get("skipped", 0))
        time_attr = testsuite.get("time", "0")
        try:
            duration = float(time_attr)
        except ValueError:
            duration = 0.0

        passed = total - failures - errors - skipped

        test_cases = []
        for tc in testsuite.findall("testcase"):
            name = tc.get("name")
            classname = tc.get("classname")
            tc_time_attr = tc.get("time", "0")
            try:
                tc_time = float(tc_time_attr)
            except ValueError:
                tc_time = 0.0

            status = "passed"
            message = None

            failure = tc.find("failure")
            if failure is not None:
                status = "failed"
                message = failure.text

            error = tc.find("error")
            if error is not None:
                status = "error"
                message = error.text

            skip = tc.find("skipped")
            if skip is not None:
                status = "skipped"
                message = skip.get("message") or skip.text

            test_cases.append(
                TestCaseResult(
                    name=name,
                    classname=classname,
                    time=tc_time,
                    status=status,
                    message=message,
                )
            )

        return TestRun(
            timestamp=datetime.now().isoformat(),
            status="failed" if (failures > 0 or errors > 0) else "passed",
            total=total,
            passed=passed,
            failed=failures,
            skipped=skipped,
            errors=errors,
            duration=duration,
            tests=test_cases,
        )
    except Exception as e:
        print(f"Error parsing junit xml: {e}")
        import traceback

        traceback.print_exc()
        return None


def main():
    junit_path = "test_output/junit.xml"
    new_run = parse_junit(junit_path)

    if not new_run:
        print("No test results found to persist.")
        return

    # Load existing history
    history = []
    if os.path.exists(HISTORY_FILE):
        try:
            with open(HISTORY_FILE, "r") as f:
                data = json.load(f)
                if isinstance(data, dict):
                    history = data.get("runs", [])
                elif isinstance(data, list):
                    history = data
        except Exception as e:
            print(f"Error loading history: {e}")
            history = []

    # Add new run
    history.insert(0, new_run.model_dump())

    # Prune and archive
    if len(history) > 5:
        to_archive = history[5:]
        history = history[:5]

        # Archive is a JSON list of runs
        archive = []
        if os.path.exists(ARCHIVE_FILE):
            try:
                with open(ARCHIVE_FILE, "r") as f:
                    archive = json.load(f)
                    if not isinstance(archive, list):
                        archive = []
            except Exception as e:
                print(f"Error loading archive: {e}")
                archive = []

        # Prepend or append? Usually archive grows at the end
        archive.extend(to_archive)

        with open(ARCHIVE_FILE, "w") as f:
            json.dump(archive, f, indent=2)
        print(f"Archived {len(to_archive)} runs to {ARCHIVE_FILE}")

    # Save history
    with open(HISTORY_FILE, "w") as f:
        json.dump({"runs": history}, f, indent=2)

    print(f"Test results (latest {len(history)}) persisted to {HISTORY_FILE}")


if __name__ == "__main__":
    main()
