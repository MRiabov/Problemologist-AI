import os
import signal
import subprocess
import sys


def run_with_timeout(cmd, timeout):
    """
    Runs a command with a timeout and ensures the whole process tree is killed
    on timeout.
    """
    # Use start_new_session to create a process group
    proc = subprocess.Popen(cmd, start_new_session=True)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"Process {proc.pid} timed out after {timeout}s. Killing group...")
        # Kill the whole process group
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGKILL)
        sys.exit(124)  # Standard exit code for timeout
    sys.exit(proc.returncode)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python timeout_wrapper.py <timeout_sec> <cmd> [args...]")
        print("Example: python timeout_wrapper.py 5 python workspace_test/loop.py")
        sys.exit(1)

    try:
        timeout = int(sys.argv[1])
    except ValueError:
        print("Timeout must be an integer.")
        sys.exit(1)

    cmd = sys.argv[2:]
    run_with_timeout(cmd, timeout)
