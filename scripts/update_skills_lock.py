from __future__ import annotations

import json
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

ROOT = Path(__file__).resolve().parents[1]
SKILLS_DIR = ROOT / "skills"
LOCK_PATH = ROOT / "config" / "skills_repo.lock.json"


def _run_git(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        ["git", "-C", str(SKILLS_DIR), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if check and result.returncode != 0:
        stderr = result.stderr.strip()
        raise RuntimeError(stderr or f"git {' '.join(args)} failed")
    return result


def _sanitize_remote_url(url: str) -> str:
    if not url:
        return ""
    if "@" not in url or "://" not in url:
        return url

    split = urlsplit(url)
    host = split.hostname or ""
    if split.port:
        host = f"{host}:{split.port}"
    return urlunsplit((split.scheme, host, split.path, split.query, split.fragment))


def _write_lock_file(payload: dict[str, str]) -> None:
    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    LOCK_PATH.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def main() -> int:
    if not (SKILLS_DIR / ".git").exists():
        print(
            "No git-managed skills repo found at skills/; skipping skills lock update."
        )
        return 0

    status = _run_git("status", "--porcelain", check=False)
    if status.returncode != 0:
        print(status.stderr.strip(), file=sys.stderr)
        return status.returncode

    dirty_output = status.stdout.strip()
    if dirty_output:
        print(
            "skills/ has uncommitted changes. Commit or stash them in the skills repo "
            "before committing the main repo.",
            file=sys.stderr,
        )
        print(dirty_output, file=sys.stderr)
        return 1

    head_commit = _run_git("rev-parse", "HEAD").stdout.strip()
    branch = _run_git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
    remote = _run_git("remote", "get-url", "origin", check=False).stdout.strip()

    payload = {
        "skills_repo_path": "skills",
        "branch": branch,
        "head_commit": head_commit,
        "origin_url": _sanitize_remote_url(remote),
        "recorded_at_utc": datetime.now(UTC)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
    }

    current = None
    if LOCK_PATH.exists():
        current = json.loads(LOCK_PATH.read_text())

    if current != payload:
        _write_lock_file(payload)
        print(f"Updated {LOCK_PATH.relative_to(ROOT)} to {head_commit[:12]}.")
    else:
        print(f"{LOCK_PATH.relative_to(ROOT)} already matches {head_commit[:12]}.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
