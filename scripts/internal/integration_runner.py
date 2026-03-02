#!/usr/bin/env python3
"""Internal integration-test orchestration utilities."""

from __future__ import annotations

import argparse
import asyncio
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
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


@dataclass
class StartedProcess:
    name: str
    process: subprocess.Popen[bytes]


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

    process = subprocess.Popen(cmd, cwd=_repo_root(), env=env)

    def _forward_signal(sig: int, _: object) -> None:
        if process.poll() is None:
            process.send_signal(sig)

    signal.signal(signal.SIGINT, _forward_signal)
    signal.signal(signal.SIGTERM, _forward_signal)

    return process.wait()


def _run(
    cmd: list[str],
    *,
    check: bool = True,
    env: dict[str, str] | None = None,
    cwd: Path | None = None,
    stdout: int | None = None,
    stderr: int | None = None,
) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        cmd,
        check=check,
        cwd=cwd or _repo_root(),
        env=env,
        stdout=stdout,
        stderr=stderr,
    )


def _parse_check(definition: str, *, kind: CheckKind) -> HealthCheck:
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


def _load_env_file(path: Path) -> None:
    if not path.exists():
        return

    print("Loading environment from .env...")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key or not (key[0].isalpha() or key[0] == "_"):
            continue
        if any(not (ch.isalnum() or ch == "_") for ch in key):
            continue

        if value and value[0] in {"'", '"'} and value[-1:] == value[0]:
            value = value[1:-1]
        else:
            hash_index = value.find("#")
            if hash_index != -1:
                value = value[:hash_index].rstrip()

        os.environ[key] = value


def _ensure_postgres_database(db_name: str) -> None:
    escaped_db_name = db_name.replace("'", "''")
    ensure_cmd = (
        "psql -U postgres -d postgres -tAc "
        f"\"SELECT 1 FROM pg_database WHERE datname='{escaped_db_name}'\" "
        f'| grep -q 1 || psql -U postgres -d postgres -c "CREATE DATABASE {db_name}"'
    )
    _run(
        [
            "docker",
            "compose",
            "-f",
            "docker-compose.test.yaml",
            "exec",
            "-T",
            "postgres",
            "sh",
            "-lc",
            ensure_cmd,
        ]
    )


def _normalize_pytest_args(pytest_args: list[str]) -> tuple[list[str], bool, bool]:
    has_marker = False
    has_file = False

    for arg in pytest_args:
        if arg == "-m":
            has_marker = True
        if arg.startswith("tests/") or "/tests/" in arg:
            has_file = True

    normalized = list(pytest_args)
    if not has_marker and not has_file:
        if len(normalized) == 1:
            normalized = ["-m", normalized[0]]
        elif len(normalized) == 0:
            normalized = [
                "-m",
                "integration_p0 or integration_p1 or integration_p2 or integration_frontend",
            ]

    if not has_file:
        normalized.extend(["tests/integration", "tests/e2e"])

    return normalized, has_marker, has_file


def _should_run_playwright(pytest_args: list[str]) -> bool:
    for arg in pytest_args:
        if "integration_frontend" in arg:
            return True
        if arg.startswith("tests/e2e") or "/tests/e2e" in arg:
            return True
        if (
            arg.startswith("tests/integration/frontend")
            or "/tests/integration/frontend" in arg
        ):
            return True
    return False


def _git_output(args: list[str]) -> str:
    completed = _run(
        args, check=False, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )
    if completed.returncode != 0:
        return ""
    return completed.stdout.decode("utf-8", errors="ignore").strip()


def _is_non_test_frontend_source_file(file_path: str) -> bool:
    if not file_path.startswith("frontend/"):
        return False
    if ".test." in file_path or ".spec." in file_path:
        return False
    if "/__tests__/" in file_path or "/tests/" in file_path:
        return False
    return True


def _has_non_test_frontend_changes(files: list[str]) -> bool:
    return any(_is_non_test_frontend_source_file(item) for item in files if item)


def _should_rebuild_frontend(frontend_state_file: Path | None) -> bool:
    repo_root = _repo_root()
    dist_dir = repo_root / "frontend" / "dist"
    if not dist_dir.exists():
        return True

    unstaged = _git_output(
        ["git", "diff", "--name-only", "--", "frontend"]
    ).splitlines()
    staged = _git_output(
        ["git", "diff", "--cached", "--name-only", "--", "frontend"]
    ).splitlines()
    untracked = _git_output(
        ["git", "ls-files", "--others", "--exclude-standard", "--", "frontend"]
    ).splitlines()
    if _has_non_test_frontend_changes(sorted(set(unstaged + staged + untracked))):
        return True

    if frontend_state_file is None or not frontend_state_file.exists():
        return True

    last_built_commit = frontend_state_file.read_text(encoding="utf-8").strip()
    if not last_built_commit:
        return True

    if (
        _run(
            ["git", "cat-file", "-e", f"{last_built_commit}^{{commit}}"], check=False
        ).returncode
        != 0
    ):
        return True

    changed_since = _git_output(
        ["git", "diff", "--name-only", f"{last_built_commit}..HEAD", "--", "frontend"]
    ).splitlines()
    return _has_non_test_frontend_changes(changed_since)


def _start_process(
    name: str,
    cmd: list[str],
    *,
    log_file: Path,
    env_updates: dict[str, str] | None = None,
    pid_file: Path | None = None,
) -> StartedProcess:
    env = os.environ.copy()
    if env_updates:
        env.update(env_updates)

    log_file.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_file, "ab")
    process = subprocess.Popen(
        cmd, cwd=_repo_root(), env=env, stdout=handle, stderr=subprocess.STDOUT
    )
    handle.close()

    if pid_file is not None:
        pid_file.parent.mkdir(parents=True, exist_ok=True)
        pid_file.write_text(f"{process.pid}\n", encoding="utf-8")

    print(f"{name} started (PID: {process.pid})")
    return StartedProcess(name=name, process=process)


def _archive_log_dir(log_dir: Path, archive_dir: Path) -> None:
    archive_dir.mkdir(parents=True, exist_ok=True)
    if log_dir.exists() and any(log_dir.iterdir()):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        archive_target = archive_dir / f"run_{timestamp}"
        shutil.move(str(log_dir), str(archive_target))
    log_dir.mkdir(parents=True, exist_ok=True)

    now = time.time()
    cutoff_seconds = 24 * 60 * 60
    for child in archive_dir.glob("run_*"):
        try:
            if now - child.stat().st_mtime > cutoff_seconds:
                if child.is_dir():
                    shutil.rmtree(child, ignore_errors=True)
                else:
                    child.unlink(missing_ok=True)
        except OSError:
            continue


def _preemptive_cleanup() -> None:
    patterns = [
        "uvicorn.*18000",
        "uvicorn.*18001",
        "uvicorn.*18002",
        "python3 -m http.server 15173",
        "python -m controller.temporal_worker",
        "npm run dev",
        "vite",
        "npx serve",
        "Xvfb",
    ]
    for pattern in patterns:
        _run(["pkill", "-f", pattern], check=False)


def _final_force_cleanup() -> None:
    patterns = [
        "uvicorn.*18000",
        "uvicorn.*18001",
        "uvicorn.*18002",
        "python -m controller.temporal_worker",
        "uv run uvicorn",
        "vite",
        "npx serve",
    ]
    for pattern in patterns:
        _run(["pkill", "-9", "-f", pattern], check=False)


def _stop_processes(processes: list[StartedProcess]) -> None:
    for started in processes:
        if started.process.poll() is None:
            started.process.terminate()

    deadline = time.time() + 5
    for started in processes:
        if started.process.poll() is not None:
            continue
        timeout = max(0.0, deadline - time.time())
        try:
            started.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            started.process.kill()


def _link_current_logs(run_playwright: bool) -> None:
    repo_root = _repo_root()
    logs_root = repo_root / "logs"
    links = [
        ("integration_tests/controller.log", "controller.log"),
        ("integration_tests/controller_debug.log", "controller_debug.log"),
        ("integration_tests/worker_light.log", "worker_light.log"),
        ("integration_tests/worker_light_debug.log", "worker_light_debug.log"),
        ("integration_tests/worker_heavy.log", "worker_heavy.log"),
        ("integration_tests/worker_heavy_debug.log", "worker_heavy_debug.log"),
        ("integration_tests/temporal_worker.log", "temporal_worker.log"),
        ("integration_tests/temporal_worker_debug.log", "temporal_worker_debug.log"),
    ]
    if run_playwright:
        links.extend(
            [
                ("integration_tests/frontend.log", "frontend.log"),
                ("integration_tests/browser_console.log", "browser_console.log"),
            ]
        )

    logs_root.mkdir(parents=True, exist_ok=True)
    for target, link_name in links:
        link_path = logs_root / link_name
        try:
            if link_path.exists() or link_path.is_symlink():
                link_path.unlink()
            link_path.symlink_to(target)
        except OSError:
            continue


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


def _run_integration_command(
    args: argparse.Namespace, passthrough_pytest_args: list[str]
) -> int:
    repo_root = _repo_root()
    os.chdir(repo_root)

    print(f"Integration Tests Started at: {time.ctime()}")

    os.environ["IS_INTEGRATION_TEST"] = "true"
    os.environ.setdefault("LOG_LEVEL", "INFO")
    os.environ["PYTHONUNBUFFERED"] = "1"

    integration_db_name = "problemologist_integration"
    integration_db_url = (
        f"postgresql+asyncpg://postgres:postgres@127.0.0.1:15432/{integration_db_name}"
    )

    os.environ["TEMPORAL_URL"] = "127.0.0.1:17233"
    os.environ["S3_ENDPOINT"] = "http://127.0.0.1:19000"
    os.environ["S3_ENDPOINT_URL"] = "http://127.0.0.1:19000"
    os.environ["S3_ACCESS_KEY"] = "minioadmin"
    os.environ["S3_SECRET_KEY"] = "minioadmin"
    os.environ["AWS_ACCESS_KEY_ID"] = "minioadmin"
    os.environ["AWS_SECRET_ACCESS_KEY"] = "minioadmin"
    os.environ["AWS_DEFAULT_REGION"] = "us-east-1"
    os.environ["WORKER_URL"] = "http://127.0.0.1:18001"
    os.environ["WORKER_HEAVY_URL"] = "http://127.0.0.1:18002"
    os.environ["ASSET_S3_BUCKET"] = "problemologist"
    os.environ["BENCHMARK_SOURCE_BUCKET"] = "benchmarks-source"
    os.environ["BENCHMARK_ASSETS_BUCKET"] = "benchmarks-assets"
    os.environ["GENESIS_FORCE_CPU"] = "1"

    sessions_dir = tempfile.mkdtemp(prefix="pb-sessions-")
    os.environ["WORKER_SESSIONS_DIR"] = sessions_dir
    print(f"Shared sessions directory: {sessions_dir}")

    if args.no_smoke:
        print("High-fidelity simulation ENABLED (smoke test mode DISABLED)")
        os.environ["SMOKE_TEST_MODE"] = "false"

    _load_env_file(repo_root / ".env")

    # Keep integration-test infra deterministic even when .env defines DB URLs.
    os.environ["POSTGRES_URL"] = integration_db_url
    os.environ["DATABASE_URL"] = integration_db_url

    pytest_args, _, _ = _normalize_pytest_args(passthrough_pytest_args)
    run_playwright = _should_run_playwright(pytest_args)

    _run(["bash", "scripts/ensure_docker_vfs.sh"])
    _run(["bash", "scripts/ensure_ngspice.sh"])

    if (
        not (repo_root / "parts.db").exists()
        or (repo_root / "parts.db").stat().st_size == 0
    ):
        print("parts.db missing or empty. Populating COTS database...")
        env = os.environ.copy()
        env["PYTHONPATH"] = "."
        _run(["uv", "run", "python", "-m", "shared.cots.indexer"], env=env)

    print("Spinning up infrastructure (Postgres, Temporal, Minio)...")
    _run(["docker", "compose", "-f", "docker-compose.test.yaml", "up", "-d"])

    print("Waiting for infra to be ready...")
    infra_checks = [
        HealthCheck(
            name="postgres",
            kind="cmd",
            target="docker compose -f docker-compose.test.yaml exec postgres pg_isready -U postgres",
        ),
        HealthCheck(
            name="minio",
            kind="http",
            target="http://127.0.0.1:19000/minio/health/live",
        ),
        HealthCheck(name="temporal", kind="tcp", target="127.0.0.1:17233"),
    ]
    asyncio.run(
        wait_for_checks_parallel(
            infra_checks,
            timeout_s=120.0,
            interval_s=0.5,
            probe_timeout_s=1.0,
        )
    )

    print("Infrastructure is up. Giving Temporal a few seconds to settle...")
    time.sleep(5)

    print(f"Ensuring integration database exists ({integration_db_name})...")
    _ensure_postgres_database(integration_db_name)

    print("Running migrations...")
    _run(["uv", "run", "alembic", "upgrade", "head"])

    print("Starting Application Servers (Controller, Worker, Temporal Worker)...")

    log_dir = repo_root / "logs" / "integration_tests"
    archive_dir = repo_root / "logs" / "archives"
    _archive_log_dir(log_dir, archive_dir)
    _preemptive_cleanup()

    processes: list[StartedProcess] = []
    frontend_pid_path = repo_root / "logs" / "frontend.pid"

    try:
        print("Starting Xvfb for headless rendering...")
        xvfb = subprocess.Popen(
            ["Xvfb", ":99", "-screen", "0", "1024x768x24"],
            cwd=repo_root,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        processes.append(StartedProcess(name="Xvfb", process=xvfb))
        os.environ["DISPLAY"] = ":99"

        time.sleep(2)
        if xvfb.poll() is not None:
            print("Xvfb failed to start. Headless rendering might fail.")
        else:
            print(f"Xvfb started (PID: {xvfb.pid}) on :99")

        processes.append(
            _start_process(
                "Worker Light",
                [
                    "uv",
                    "run",
                    "uvicorn",
                    "worker_light.app:app",
                    "--host",
                    "0.0.0.0",
                    "--port",
                    "18001",
                ],
                log_file=log_dir / "worker_light.log",
                env_updates={
                    "WORKER_TYPE": "light",
                    "EXTRA_DEBUG_LOG": str(log_dir / "worker_light_debug.log"),
                },
                pid_file=repo_root / "logs" / "worker_light.pid",
            )
        )

        processes.append(
            _start_process(
                "Worker Heavy",
                [
                    "uv",
                    "run",
                    "uvicorn",
                    "worker_heavy.app:app",
                    "--host",
                    "0.0.0.0",
                    "--port",
                    "18002",
                ],
                log_file=log_dir / "worker_heavy.log",
                env_updates={
                    "WORKER_TYPE": "heavy",
                    "EXTRA_DEBUG_LOG": str(log_dir / "worker_heavy_debug.log"),
                },
                pid_file=repo_root / "logs" / "worker_heavy.pid",
            )
        )

        processes.append(
            _start_process(
                "Controller",
                [
                    "uv",
                    "run",
                    "uvicorn",
                    "controller.api.main:app",
                    "--host",
                    "0.0.0.0",
                    "--port",
                    "18000",
                ],
                log_file=log_dir / "controller.log",
                env_updates={"EXTRA_DEBUG_LOG": str(log_dir / "controller_debug.log")},
                pid_file=repo_root / "logs" / "controller.pid",
            )
        )

        pythonpath = os.environ.get("PYTHONPATH", "")
        combined_pythonpath = f"{pythonpath}:." if pythonpath else "."
        processes.append(
            _start_process(
                "Temporal Worker",
                ["uv", "run", "python", "-m", "controller.temporal_worker"],
                log_file=log_dir / "temporal_worker.log",
                env_updates={
                    "PYTHONPATH": combined_pythonpath,
                    "EXTRA_DEBUG_LOG": str(log_dir / "temporal_worker_debug.log"),
                },
                pid_file=repo_root / "logs" / "temporal_worker.pid",
            )
        )

        if run_playwright:
            git_dir = _git_output(["git", "rev-parse", "--git-dir"])
            frontend_state_file = (
                Path(git_dir) / "problemologist_integration_frontend_build_commit"
                if git_dir
                else None
            )

            if _should_rebuild_frontend(frontend_state_file):
                print(
                    "Building frontend (detected non-test frontend changes since last integration build or missing dist)..."
                )
                env = os.environ.copy()
                env["PYTHONPATH"] = "."
                _run(["uv", "run", "python", "scripts/generate_openapi.py"], env=env)
                _run(["npm", "run", "gen:api"], cwd=repo_root / "frontend")
                production_env = repo_root / "frontend" / ".env.production"
                production_env.write_text(
                    "VITE_API_URL=http://localhost:18000\nVITE_IS_INTEGRATION_TEST=true\n",
                    encoding="utf-8",
                )
                shutil.rmtree(repo_root / "frontend" / "dist", ignore_errors=True)
                _run(["npm", "run", "build"], cwd=repo_root / "frontend")
                if frontend_state_file is not None:
                    frontend_state_file.parent.mkdir(parents=True, exist_ok=True)
                    frontend_state_file.write_text(
                        _git_output(["git", "rev-parse", "HEAD"]) + "\n",
                        encoding="utf-8",
                    )
            else:
                print(
                    "Skipping frontend build (no non-test frontend changes since last integration build)."
                )

            processes.append(
                _start_process(
                    "Frontend server",
                    [
                        "npx",
                        "http-server",
                        "-p",
                        "15173",
                        "--proxy",
                        "http://localhost:18000?",
                    ],
                    log_file=log_dir / "frontend.log",
                    pid_file=frontend_pid_path,
                )
            )
            print("Frontend server available on http://localhost:15173")

        _link_current_logs(run_playwright)

        print("Waiting for servers to be healthy...")
        server_checks = [
            HealthCheck(
                name="controller",
                kind="http",
                target="http://127.0.0.1:18000/health",
                contains="healthy",
            ),
            HealthCheck(
                name="worker-light",
                kind="http",
                target="http://127.0.0.1:18001/health",
                contains="healthy",
            ),
            HealthCheck(
                name="worker-heavy",
                kind="http",
                target="http://127.0.0.1:18002/health",
                contains="healthy",
            ),
        ]
        if run_playwright:
            server_checks.append(
                HealthCheck(
                    name="frontend", kind="http", target="http://127.0.0.1:15173"
                )
            )

        asyncio.run(
            wait_for_checks_parallel(
                server_checks,
                timeout_s=120.0,
                interval_s=0.5,
                probe_timeout_s=1.0,
            )
        )

        for started in processes:
            if started.name == "Frontend server" and not run_playwright:
                continue
            if started.process.poll() is not None:
                print(f"{started.name} died unexpectedly!", file=sys.stderr)
                if started.name == "Temporal Worker":
                    temporal_log = log_dir / "temporal_worker.log"
                    if temporal_log.exists():
                        print(
                            "--- LAST 20 LINES OF TEMPORAL WORKER LOG ---",
                            file=sys.stderr,
                        )
                        lines = temporal_log.read_text(
                            encoding="utf-8", errors="ignore"
                        ).splitlines()
                        for line in lines[-20:]:
                            print(line, file=sys.stderr)
                return 1

        (repo_root / "test_output").mkdir(parents=True, exist_ok=True)
        pytest_exit = run_pytest_subprocess(
            pytest_args=pytest_args,
            reverse=args.reverse,
            junit_xml="test_output/junit.xml",
        )

        _run(["uv", "run", "python", "scripts/persist_test_results.py"], check=False)

        if pytest_exit == 0:
            print("Integration tests PASSED!")
        else:
            print("Integration tests FAILED!")

        print(f"Integration Tests Finished at: {time.ctime()}")
        return pytest_exit
    finally:
        print("Cleaning up processes...")
        _stop_processes(processes)
        _final_force_cleanup()

        for pid_file in [
            repo_root / "logs" / "controller.pid",
            repo_root / "logs" / "temporal_worker.pid",
            repo_root / "logs" / "worker_light.pid",
            repo_root / "logs" / "worker_heavy.pid",
            repo_root / "logs" / "worker.pid",
            repo_root / "logs" / "frontend.pid",
        ]:
            pid_file.unlink(missing_ok=True)

        if Path(sessions_dir).exists():
            shutil.rmtree(sessions_dir, ignore_errors=True)

        compose_cmd = ["docker", "compose", "-f", "docker-compose.test.yaml"]
        if args.down:
            print("Bringing down infrastructure containers (--down flag provided)...")
            _run(compose_cmd + ["down"], check=False)
        else:
            print("Stopping infrastructure containers...")
            _run(compose_cmd + ["stop"], check=False)


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

    run_parser = sub.add_parser("run", help="Run full integration orchestration")
    run_parser.add_argument("--reverse", action="store_true")
    run_parser.add_argument("--down", action="store_true")
    run_parser.add_argument("--no-smoke", action="store_true")

    return parser


def main() -> int:
    parser = _build_parser()
    args, unknown = parser.parse_known_args()

    if args.command == "wait":
        if unknown:
            parser.error(
                f"Unrecognized arguments for wait: {' '.join(shlex.quote(arg) for arg in unknown)}"
            )
        return _run_wait_command(args)

    if args.command == "run-pytest":
        if unknown:
            parser.error(
                f"Unrecognized arguments for run-pytest: {' '.join(shlex.quote(arg) for arg in unknown)}"
            )
        return _run_pytest_command(args)

    if args.command == "run":
        passthrough = list(unknown)
        if passthrough and passthrough[0] == "--":
            passthrough = passthrough[1:]
        return _run_integration_command(args, passthrough)

    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
