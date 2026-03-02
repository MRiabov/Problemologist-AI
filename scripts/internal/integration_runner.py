#!/usr/bin/env python3
"""Internal integration-test orchestration utilities.

This module is intentionally standalone so the shell runner can adopt pieces of
it later without changing behavior all at once.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

CheckKind = Literal["http", "tcp", "cmd"]


@dataclass(frozen=True)
class HealthCheck:
    name: str
    kind: CheckKind
    target: str
    contains: str | None = None


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _check_http(url: str, *, contains: str | None, timeout_s: float) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            if contains is None:
                return True
            payload = response.read().decode("utf-8", errors="ignore")
            return contains in payload
    except (urllib.error.URLError, TimeoutError, OSError):
        return False


def _check_tcp(host: str, port: int, *, timeout_s: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _check_cmd(command: str) -> bool:
    completed = subprocess.run(
        command,
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0


async def _check_once(check: HealthCheck, *, timeout_s: float) -> bool:
    if check.kind == "http":
        return await asyncio.to_thread(
            _check_http, check.target, contains=check.contains, timeout_s=timeout_s
        )
    if check.kind == "tcp":
        host, port_s = check.target.rsplit(":", 1)
        return await asyncio.to_thread(
            _check_tcp, host, int(port_s), timeout_s=timeout_s
        )
    return await asyncio.to_thread(_check_cmd, check.target)


async def wait_for_check(
    check: HealthCheck,
    *,
    timeout_s: float = 120.0,
    interval_s: float = 0.5,
    probe_timeout_s: float = 1.0,
) -> float:
    """Wait until a single readiness check succeeds.

    Returns elapsed seconds on success. Raises TimeoutError on failure.
    """
    started = time.monotonic()
    while True:
        if await _check_once(check, timeout_s=probe_timeout_s):
            return time.monotonic() - started
        elapsed = time.monotonic() - started
        if elapsed >= timeout_s:
            raise TimeoutError(
                f"Timed out waiting for {check.name} ({check.kind}:{check.target})"
            )
        await asyncio.sleep(interval_s)


async def wait_for_checks_parallel(
    checks: list[HealthCheck],
    *,
    timeout_s: float = 120.0,
    interval_s: float = 0.5,
    probe_timeout_s: float = 1.0,
) -> dict[str, float]:
    """Wait for all checks concurrently.

    Returns a name->elapsed map. Raises TimeoutError as soon as one check times out.
    """
    tasks = {
        check.name: asyncio.create_task(
            wait_for_check(
                check,
                timeout_s=timeout_s,
                interval_s=interval_s,
                probe_timeout_s=probe_timeout_s,
            )
        )
        for check in checks
    }
    try:
        results = await asyncio.gather(*tasks.values())
        return dict(zip(tasks.keys(), results, strict=True))
    finally:
        for task in tasks.values():
            if not task.done():
                task.cancel()


def run_pytest_subprocess(
    pytest_args: list[str],
    *,
    reverse: bool = False,
    junit_xml: str = "test_output/junit.xml",
    extra_env: dict[str, str] | None = None,
) -> int:
    """Run pytest in a child process the same way the shell runner does."""
    cmd = [
        "uv",
        "run",
        "pytest",
        "-v",
        "-o",
        "addopts=-n0",
        "--maxfail=3",
        "-s",
    ]
    if reverse:
        cmd.append("--reverse")
    cmd.extend(pytest_args)
    cmd.append(f"--junitxml={junit_xml}")

    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)

    process = subprocess.Popen(
        cmd,
        cwd=_repo_root(),
        env=env,
    )

    def _forward_signal(sig: int, _: object) -> None:
        if process.poll() is None:
            process.send_signal(sig)

    signal.signal(signal.SIGINT, _forward_signal)
    signal.signal(signal.SIGTERM, _forward_signal)

    return process.wait()


def _parse_check(definition: str, *, kind: CheckKind) -> HealthCheck:
    # HTTP format: "name|url|contains" (contains optional)
    # TCP format: "name|host:port"
    # CMD format: "name|command"
    parts = definition.split("|")
    if len(parts) < 2:
        msg = f"Invalid {kind} check definition: {definition!r}"
        raise argparse.ArgumentTypeError(msg)

    name = parts[0].strip()
    if not name:
        msg = f"Missing check name in definition: {definition!r}"
        raise argparse.ArgumentTypeError(msg)

    if kind == "http":
        target = parts[1].strip()
        contains = parts[2].strip() if len(parts) >= 3 and parts[2].strip() else None
        return HealthCheck(name=name, kind=kind, target=target, contains=contains)

    target = parts[1].strip()
    return HealthCheck(name=name, kind=kind, target=target)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Internal integration test helpers")
    sub = parser.add_subparsers(dest="command", required=True)

    wait_parser = sub.add_parser("wait", help="Run readiness checks in parallel")
    wait_parser.add_argument(
        "--http-check",
        action="append",
        default=[],
        help="HTTP check: name|url|contains (contains optional)",
    )
    wait_parser.add_argument(
        "--tcp-check",
        action="append",
        default=[],
        help="TCP check: name|host:port",
    )
    wait_parser.add_argument(
        "--cmd-check",
        action="append",
        default=[],
        help="Command check: name|command",
    )
    wait_parser.add_argument("--timeout", type=float, default=120.0)
    wait_parser.add_argument("--interval", type=float, default=0.5)
    wait_parser.add_argument("--probe-timeout", type=float, default=1.0)

    pytest_parser = sub.add_parser("run-pytest", help="Run pytest as a subprocess")
    pytest_parser.add_argument("--reverse", action="store_true")
    pytest_parser.add_argument("--junit-xml", default="test_output/junit.xml")
    pytest_parser.add_argument("pytest_args", nargs=argparse.REMAINDER)
    return parser


def _run_wait_command(args: argparse.Namespace) -> int:
    checks: list[HealthCheck] = []
    checks.extend(_parse_check(item, kind="http") for item in args.http_check)
    checks.extend(_parse_check(item, kind="tcp") for item in args.tcp_check)
    checks.extend(_parse_check(item, kind="cmd") for item in args.cmd_check)

    if not checks:
        print("No checks provided.", file=sys.stderr)
        return 2

    started = time.monotonic()
    try:
        results = asyncio.run(
            wait_for_checks_parallel(
                checks,
                timeout_s=args.timeout,
                interval_s=args.interval,
                probe_timeout_s=args.probe_timeout,
            )
        )
    except TimeoutError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    total = time.monotonic() - started
    print(f"All checks passed in {total:.2f}s")
    for name, elapsed in sorted(results.items()):
        print(f"  - {name}: {elapsed:.2f}s")
    return 0


def _run_pytest_command(args: argparse.Namespace) -> int:
    pytest_args = args.pytest_args
    if pytest_args and pytest_args[0] == "--":
        pytest_args = pytest_args[1:]
    return run_pytest_subprocess(
        pytest_args=pytest_args,
        reverse=args.reverse,
        junit_xml=args.junit_xml,
    )


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    if args.command == "wait":
        return _run_wait_command(args)
    if args.command == "run-pytest":
        return _run_pytest_command(args)

    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
