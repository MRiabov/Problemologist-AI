import os
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime

# Path configuration
PROJ_ROOT = "/home/maksym/Work/proj/Problemologist/Problemologist-AI"
AUTOMATIONS_DIR = os.path.join(PROJ_ROOT, "dev/automations")
TRACKER_FILE = os.path.join(AUTOMATIONS_DIR, "overnight_tracker.json")
LOG_FILE = os.path.join(AUTOMATIONS_DIR, "overnight_log.md")
MAX_JULES_CONCURRENT = 3


def log(message):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = f"[{timestamp}] {message}\n"
    print(entry, end="")
    with open(LOG_FILE, "a") as f:
        f.write(entry)


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


def get_current_branch():
    res = run_cmd(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    return res.stdout.strip() if res else "main"


def parse_failures_xml(xml_path="test_output/junit.xml"):
    """Parses JUnit XML to extract failures with details."""
    failures = []
    if not os.path.exists(xml_path):
        return []

    try:
        tree = ET.parse(xml_path)
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


def start_jules_session(branch_name, failures):
    # failures is now a list of dicts: {"id": "...", "details": "..."}
    # Ensure failure list is not too long to avoid API 400 errors

    # Format details for prompt
    failure_descriptions = []
    for f in failures[:5]:  # Truncate to 5
        failure_descriptions.append(f"Test: {f['id']}\n{f['details']}")

    failure_text = "\n---\n".join(failure_descriptions)

    prompt = f"""The following integration tests are failing on branch {branch_name}. 
Please fix the underlying code. Refer to @specs/integration-tests.md and @specs/desired_architecture.md.

FAILURES:
{failure_text}
"""
    log(
        f"Starting Jules session for {len(failures)} failures (truncated to 5 for API)..."
    )
    # Wrap prompt in single quotes for shell safety if it contains double quotes
    safe_prompt = prompt.replace('"', '\\"')
    # Using subprocess to run jules command
    cmd = f'jules new "{safe_prompt}"'
    subprocess.Popen(cmd, shell=True, cwd=PROJ_ROOT, start_new_session=True)


def trigger_gemini_fix(branch_name, failures, reason=""):
    log(
        f"URGENT: Triggering Gemini CLI fix for {len(failures)} failures. Reason: {reason}"
    )

    # Format details for prompt
    failure_descriptions = []
    for f in failures:
        failure_descriptions.append(f"Test: {f['id']}\n{f['details']}")
    failure_text = "\n---\n".join(failure_descriptions)

    prompt = f"""URGENT FIX REQUIRED on branch {branch_name}. 
The following integration tests are failing and Jules was unable to resolve them or they are blocking progress.
{reason}

FAILURES:
{failure_text}

Please analyze the codebase, identify the missing files or logic errors, and apply a fix immediately. 
Use --yolo to proceed with the fix. Ensure high technical rigor."""

    # Run Gemini CLI for immediate fix
    run_cmd(["gemini", "--yolo", "-p", prompt])
    log("Gemini CLI fix attempt complete.")


def monitor_jules_and_merge(branch_name, current_failures):
    """
    Checks 'jules remote list --session' to see if any are finished.
    If finished, pulls and merges using Gemini CLI.
    """
    res = run_cmd(["jules", "remote", "list", "--session"])
    if not res:
        return

    lines = res.stdout.splitlines()
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        session_id = parts[0]

        if any(status in line for status in ["Finished", "Completed"]):
            log(f"Jules session {session_id} finished. Applying and merging...")
            run_cmd(["jules", "remote", "pull", "--session", session_id, "--apply"])

            merge_prompt = f"""I have applied changes from Jules session {session_id}. 
Review these changes, ensure they align with @specs/integration-tests.md and @specs/desired_architecture.md.
If they look correct and fix the failures without obvious regressions, commit them to {branch_name}.
If there are minor issues, fix them. Use --yolo for speed."""

            run_cmd(["gemini", "--yolo", "-p", merge_prompt])
            log(f"Merged Jules session {session_id} into {branch_name}")

        elif "Failed" in line:
            log(f"Jules session {session_id} FAILED. Escalating to Gemini...")
            # Escalate the specific failures assigned to this session
            # For now, we take the current set of failures as context
            trigger_gemini_fix(
                branch_name,
                current_failures[:5],
                reason=f"Jules session {session_id} failed.",
            )


def main():
    # Setup overnight branch
    start_time = datetime.now().strftime("%b-%d-%H-%M")
    overnight_branch = f"overnight-{start_time}"
    log(f"Initializing overnight run on branch: {overnight_branch}")

    run_cmd(["git", "checkout", "-b", overnight_branch])

    while True:
        log("--- Starting New Iteration ---")

        # Check if we should continue indefinitely (True by default for overnight)
        # We will loop until manually stopped or until no failures exist.

        # 1. Run integration tests (limit to 15 failures to save time/resources)
        # Note: Resource usage is high, so we run this sequentially.
        log("Executing integration tests...")
        test_res = run_cmd(
            [
                "bash",
                "scripts/run_integration_tests.sh",
                "--maxfail=15",
                "tests/integration",
            ]
        )

        if test_res and test_res.returncode != 0:
            log(f"Test run finished with exit code {test_res.returncode}")

        # Check for critical stdout errors first
        if test_res and "pytest: error: argument -m" in test_res.stdout:
            log(
                "CRITICAL: Pytest failed due to empty marker argument. Forcing default markers."
            )
            failures = [{"id": "FORCE_RUN_ALL", "details": "Marker error"}]
        else:
            # Try to parse XML first for rich details
            failures = parse_failures_xml("test_output/junit.xml")
            if not failures and test_res:
                # Fallback to stdout scraping if XML is empty but return code non-zero?
                # Actually, parse_failures_stdout logic handles scraping if XML failed.
                # But we want to prefer XML.
                # If parse_failures_xml returns empty, check if we scraped any IDs from stdout.
                stdout_failures = parse_failures(test_res.stdout)
                if stdout_failures:
                    failures = stdout_failures

        if failures and failures[0]["id"] == "FORCE_RUN_ALL":
            log("Rerunning tests without markers...")
            test_res = run_cmd(
                [
                    "bash",
                    "scripts/run_integration_tests.sh",
                    "--maxfail=15",
                    "tests/integration",
                ]
            )
            # Re-parse
            failures = parse_failures_xml("test_output/junit.xml")
            if not failures and test_res:
                failures = parse_failures(test_res.stdout)

        log(f"Iteration complete. {len(failures)} failures detected.")

        if not failures:
            log("No failures detected. Sleeping for 1 hour.")
            time.sleep(3600)
            continue

        # 2. Split failures and assign to Jules (up to MAX_JULES_CONCURRENT)
        # Assuming we split failures into batches of ~5 to keep Jules focused
        batch_size = 5
        for i in range(
            0, min(len(failures), batch_size * MAX_JULES_CONCURRENT), batch_size
        ):
            batch = failures[i : i + batch_size]
            start_jules_session(overnight_branch, batch)

        # 3. Monitor and Merge loop
        # We wait for sessions to finish
        log("Waiting for Jules sessions to complete...")
        for _ in range(12):  # Wait up to 2 hours (12 * 10 mins)
            time.sleep(600)
            monitor_jules_and_merge(overnight_branch, failures)

            # Periodic status update to the user (every 2 hours)
            # This logic will be handled by the main orchestrator reporting back via Telegram

        # 4. Final check for the iteration
        log("Iteration check-in complete.")


if __name__ == "__main__":
    main()
