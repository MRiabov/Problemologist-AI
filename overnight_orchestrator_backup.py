import json
import subprocess
import os
import sys
import time
import re
from datetime import datetime

# Path configuration
PROJ_ROOT = "/home/maksym/Work/proj/Problemologist/Problemologist-AI"
TRACKER_FILE = os.path.join(PROJ_ROOT, "overnight_tracker.json")
LOG_FILE = os.path.join(PROJ_ROOT, "overnight_log.md")
MAX_JULES_CONCURRENT = 3


def log(message):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = f"[{timestamp}] {message}\n"
    print(entry, end="")
    with open(LOG_FILE, "a") as f:
        f.write(entry)


def run_cmd(cmd, cwd=PROJ_ROOT, capture=True):
    try:
        result = subprocess.run(
            cmd, cwd=cwd, capture_output=capture, text=True, shell=isinstance(cmd, str)
        )
        return result
    except Exception as e:
        log(f"Error running command {cmd}: {e}")
        return None


def get_current_branch():
    res = run_cmd(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    return res.stdout.strip() if res else "main"


def parse_failures(output):
    """Extracts failed test names from pytest output."""
    failures = []
    # Look for lines like: FAILED tests/integration/test_worker_concurrency.py::test_worker_concurrency - ...
    pattern = re.compile(r"FAILED\s+(tests/integration/[^ ]+)")
    for line in output.splitlines():
        match = pattern.search(line)
        if match:
            failures.append(match.group(1))
    return list(set(failures))


def start_jules_session(branch_name, failures):
    failure_text = "\n".join(failures)
    prompt = f"""The following integration tests are failing on branch {branch_name}. 
Please fix the underlying code. Ensure no regressions in other existing tests.
Refer to @specs/integration-tests.md and @specs/desired_architecture.md for requirements.

FAILURES:
{failure_text}
"""
    log(f"Starting Jules session for {len(failures)} failures...")
    # Using 'jules new' with --branch if supported or rely on current branch
    # Assuming Jules picks up the current branch.
    cmd = f'jules new "{prompt}"'
    # We run this in background or separate process as Jules TUI/CLI might hang
    subprocess.Popen(cmd, shell=True, cwd=PROJ_ROOT, start_new_session=True)


def monitor_jules_and_merge(branch_name):
    """
    Checks 'jules remote list --session' to see if any are finished.
    If finished, pulls and merges using Gemini CLI.
    """
    res = run_cmd(["jules", "remote", "list", "--session"])
    if not res:
        return

    # Simple parsing of jules list: look for IDs and status
    # This is a placeholder as the exact output format of 'jules remote list' isn't fully known
    # but typically includes a table with ID and Status.
    lines = res.stdout.splitlines()
    for line in lines:
        if "Finished" in line or "Completed" in line:
            # Extract ID
            session_id = line.split()[0]
            log(f"Jules session {session_id} finished. Applying and merging...")

            # 1. Pull changes
            run_cmd(["jules", "remote", "pull", "--session", session_id, "--apply"])

            # 2. Use Gemini CLI to help with the merge/commit and validation
            merge_prompt = f"""I have applied changes from Jules session {session_id}. 
Review these changes, ensure they align with @specs/integration-tests.md and @specs/desired_architecture.md.
If they look correct and fix the failures without obvious regressions, commit them to {branch_name}.
If there are minor issues, fix them. Use --yolo for speed but be technically rigorous."""

            run_cmd(["gemini", "--yolo", "-p", merge_prompt])
            log(f"Merged Jules session {session_id} into {branch_name}")


def main():
    # Setup overnight branch
    start_time = datetime.now().strftime("%b-%d-%H-%M")
    overnight_branch = f"overnight-{start_time}"
    log(f"Initializing overnight run on branch: {overnight_branch}")

    run_cmd(["git", "checkout", "-b", overnight_branch])

    while True:
        log("--- Starting New Iteration ---")

        # 1. Run integration tests (limit to 15 failures to save time/resources)
        # Note: Resource usage is high, so we run this sequentially.
        test_res = run_cmd(["bash", "scripts/run_integration_tests.sh", "--maxfail=15"])

        failures = parse_failures(test_res.stdout) if test_res else []
        log(f"Test run completed. {len(failures)} failures detected.")

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
            monitor_jules_and_merge(overnight_branch)

            # Periodic status update to the user (every 2 hours)
            # This logic will be handled by the main orchestrator reporting back via Telegram

        # 4. Final check for the iteration
        log("Iteration check-in complete.")


if __name__ == "__main__":
    main()
