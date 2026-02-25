import json
import os
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta

# Path configuration
PROJ_ROOT = "/home/maksym/Work/proj/Problemologist/Problemologist-AI"
AUTOMATIONS_DIR = os.path.join(PROJ_ROOT, "dev/automations")
STATE_FILE = os.path.join(AUTOMATIONS_DIR, "nexus_state.json")
LOG_FILE = os.path.join(AUTOMATIONS_DIR, "nexus_log.md")
PULSE_FILE = os.path.join(AUTOMATIONS_DIR, "nexus_pulse.md")
MAX_JULES_CONCURRENT = 3


def log(message):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = f"[{timestamp}] {message}\n"
    print(entry, end="")
    with open(LOG_FILE, "a") as f:
        f.write(entry)


def update_pulse():
    state = get_state()
    four_hours_ago = datetime.now() - timedelta(hours=4)
    recent_merges = [
        m for m in state["merges"] if datetime.fromisoformat(m["time"]) > four_hours_ago
    ]

    pulse = f"""# Nexus Orchestrator Pulse
Last Update: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
Uptime: {datetime.now() - datetime.fromisoformat(state.get("start_time", datetime.now().isoformat()))!s}

## KPIs (Last 4 Hours)
- **Merges:** {len(recent_merges)} (Target: 3-5)
- **Active Jules Sessions:** {len(state.get("initiated_sessions", []))}

## System Status
- **Tests Running:** {check_process("pytest")}
- **Branch:** {run_cmd(["git", "rev-parse", "--abbrev-ref", "HEAD"]).stdout.strip() if os.path.exists(os.path.join(PROJ_ROOT, ".git")) else "unknown"}

## Recent Merges
"""
    for m in recent_merges:
        pulse += f"- {m['id']} at {m['time']}\n"

    with open(PULSE_FILE, "w") as f:
        f.write(pulse)


def check_process(name):
    res = run_cmd(f"pgrep -f {name}")
    return bool(res and res.stdout.strip())


def run_cmd(cmd, cwd=PROJ_ROOT, capture=True):
    try:
        stdout_dest = subprocess.PIPE if capture else None
        stderr_dest = subprocess.STDOUT if capture else None
        result = subprocess.run(
            cmd,
            cwd=cwd,
            stdout=stdout_dest,
            stderr=stderr_dest,
            text=True,
            shell=isinstance(cmd, str),
        )
        return result
    except Exception as e:
        log(f"Error running command {cmd}: {e}")
        return None


def get_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE) as f:
            try:
                return json.load(f)
            except:
                pass
    return {
        "initiated_sessions": [],
        "merges": [],
        "start_time": datetime.now().isoformat(),
    }


def save_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)


def parse_failures_xml(xml_path="test_output/junit.xml"):
    """Parses JUnit XML to extract failures with details."""
    failures = []
    if not os.path.exists(os.path.join(PROJ_ROOT, xml_path)):
        return []

    try:
        tree = ET.parse(os.path.join(PROJ_ROOT, xml_path))
        root = tree.getroot()
        for testcase in root.findall(".//testcase"):
            failure = testcase.find("failure")
            error = testcase.find("error")  # specific error vs assertion failure
            issue = failure if failure is not None else error

            if issue is not None:
                file_path = testcase.get("file")
                name = testcase.get("name")
                classname = testcase.get("classname")

                # Pytest junitxml usually has file path. If not, construct from classname.
                test_id = (
                    f"{file_path}::{name}" if file_path else f"{classname}::{name}"
                )

                message = issue.get("message", "No message")
                # Limit detailed trace to avoid token limits, but give enough context
                details = issue.text or ""
                details = details.strip()
                if len(details) > 1000:
                    details = details[:1000] + "\n...[truncated]..."

                failures.append(
                    {
                        "id": test_id,
                        "details": f"Error: {message}\nTraceback:\n{details}",
                    }
                )
    except Exception as e:
        log(f"Error parsing XML: {e}")

    return failures


def parse_failures(output):
    """Fallback: Extracts failed test names from pytest output (no details)."""
    failures = []
    if not output:
        return []

    # Check for the error "pytest: error: argument -m: expected one argument"
    if "pytest: error: argument -m: expected one argument" in output:
        log(
            "CRITICAL: Pytest failed due to empty marker argument. Forcing default markers."
        )
        return [{"id": "FORCE_RUN_ALL", "details": "Marker error"}]

    # Look for lines like: FAILED tests/integration/test_worker_concurrency.py::test_worker_concurrency - ...
    pattern = re.compile(r"FAILED\s+(tests/integration/[^ ]+)")
    for line in output.splitlines():
        match = pattern.search(line)
        if match:
            failures.append(
                {
                    "id": match.group(1),
                    "details": "Detail extraction failed (parsed from stdout)",
                }
            )

    # Dedup by ID
    unique = {}
    for f in failures:
        unique[f["id"]] = f
    return list(unique.values())


def main():
    log("Initializing Nexus Orchestrator...")
    state = get_state()
    state["start_time"] = datetime.now().isoformat()
    save_state(state)

    while True:
        update_pulse()

        # 1. Run integration tests
        log("Executing integration tests...")
        # Sequential run to avoid overloading
        test_res = run_cmd(
            [
                "bash",
                "scripts/run_integration_tests.sh",
                "--maxfail=15",
                "tests/integration",
            ]
        )
        # Try XML first, fallback to regex
        failures = parse_failures_xml("test_output/junit.xml")
        if not failures and test_res:
            stdout_failures = parse_failures(test_res.stdout)
            if stdout_failures:
                failures = stdout_failures

        log(f"Iteration complete. {len(failures)} failures detected.")

        if not failures:
            log("All tests passed. Sleeping for 1 hour.")
            time.sleep(3600)
            continue

        # 2. Start Jules sessions for failures
        batch_size = 5
        for i in range(
            0, min(len(failures), batch_size * MAX_JULES_CONCURRENT), batch_size
        ):
            batch = failures[i : i + batch_size]

            # Format details for prompt
            failure_descriptions = []
            for f in batch:
                failure_descriptions.append(f"Test: {f['id']}\n{f['details']}")
            failure_text = "\n---\n".join(failure_descriptions)

            prompt = f"Fix integration test failures:\n{failure_text}\nRef: @specs/integration-tests.md and @specs/desired_architecture.md."

            log(f"Initiating Jules session for {len(batch)} failures...")
            # Use 'jules new' and capture output
            res = run_cmd(f'jules new "{prompt}"')
            if res and res.returncode == 0:
                # Try to find ID in output
                match = re.search(r"Created session (\d+)", res.stdout)
                if match:
                    sid = match.group(1)
                    state = get_state()
                    state["initiated_sessions"].append(sid)
                    save_state(state)
                    log(f"Session {sid} created successfully.")
                else:
                    log(
                        f"Session created but ID not found in output. Stdout: {res.stdout.strip()}"
                    )
            else:
                log(
                    f"Failed to create Jules session: {res.stderr if res else 'No result'}"
                )

        # 3. Monitor and Merge (loop for 1 hour)
        start_monitor = datetime.now()
        while datetime.now() - start_monitor < timedelta(hours=1):
            update_pulse()
            time.sleep(600)
            log("Monitoring Jules sessions...")
            res = run_cmd(["jules", "remote", "list", "--session"])
            if not res:
                continue

            state = get_state()
            new_initiated = []

            lines = res.stdout.splitlines()
            for line in lines:
                parts = line.split()
                if not parts:
                    continue
                session_id = parts[0]

                if session_id in state["initiated_sessions"]:
                    if any(status in line for status in ["Finished", "Completed"]):
                        log(
                            f"Nexus Session {session_id} finished. Applying and merging..."
                        )
                        run_cmd(
                            [
                                "jules",
                                "remote",
                                "pull",
                                "--session",
                                session_id,
                                "--apply",
                            ]
                        )

                        commit_msg = f"chore(nexus): merge jules fix {session_id}"
                        run_cmd(
                            f'git add . && git commit --author="Nexus <nexus@openclaw.ai>" -m "{commit_msg}"'
                        )

                        state["merges"].append(
                            {"id": session_id, "time": datetime.now().isoformat()}
                        )
                        log(f"Successfully merged {session_id}")
                    else:
                        new_initiated.append(session_id)

            state["initiated_sessions"] = new_initiated
            save_state(state)


if __name__ == "__main__":
    main()
