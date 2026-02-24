import subprocess
import sys


def run_command(command):
    print(f"Running: {' '.join(command)}")
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
    else:
        print(result.stdout)
    return result


def main():
    # 0. Check for dirty working tree
    print("Checking for dirty working tree...")
    res = run_command(["git", "status", "--porcelain"])
    if res.stdout.strip():
        print(
            "Working tree is dirty. Please commit or stash your changes before updating."
        )
        sys.exit(1)

    # 1. Fetch latest from origin
    print("Fetching latest changes from origin...")
    run_command(["git", "fetch", "origin"])

    # 2. Check current branch
    res = run_command(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    current_branch = res.stdout.strip()
    print(f"Current branch: {current_branch}")

    # 3. Check for new commits on origin/main
    print("Checking for new commits on origin/main...")
    res = run_command(["git", "log", "HEAD..origin/main", "--oneline"])
    new_commits = res.stdout.strip()

    if not new_commits:
        print("No new commits on origin/main. You are up to date.")
    else:
        print(f"New commits found on origin/main:\n{new_commits}")
        print("Attempting to rebase onto origin/main...")
        res = run_command(["git", "rebase", "origin/main"])
        if res.returncode == 0:
            print("Successfully rebased onto origin/main.")

            # 4. Check if dependencies changed
            # Compare HEAD with ORIG_HEAD (the state before rebase)
            diff_res = run_command(
                ["git", "diff", "ORIG_HEAD", "HEAD", "--", "uv.lock", "pyproject.toml"]
            )
            if diff_res.stdout.strip():
                print("Dependencies might have changed. Updating...")
                # Try to update dependencies
                run_command(["uv", "pip", "install", "--system", "-e", "."])
            else:
                print("No dependency changes detected.")
        else:
            print("Rebase failed! Please resolve conflicts manually.")
            print("You can use 'git rebase --abort' to return to your previous state.")
            sys.exit(1)


if __name__ == "__main__":
    main()
