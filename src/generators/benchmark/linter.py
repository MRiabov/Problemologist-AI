import json
import subprocess
import tempfile
from pathlib import Path

import yaml

# Load config
CONFIG_PATH = Path(__file__).parent / "lint_config.yaml"
with CONFIG_PATH.open("r") as f:
    config = yaml.safe_load(f)

MAX_ERRORS = config.get("max_errors", 10)
IGNORE_WARNINGS = config.get("ignore_warnings", True)
RUFF_RULES = ",".join(config.get("ruff_rules", ["E", "F"]))
RUFF_IGNORE = ",".join(config.get("ruff_ignore", []))


def run_linter(code: str) -> list[str]:
    """Runs ruff and pyrefly on the provided code and returns formatted errors."""
    errors = []

    # 1. Run Ruff via stdin
    try:
        ruff_cmd = [
            "ruff",
            "check",
            "--stdin-filename",
            "gen.py",
            "--output-format",
            "json",
            "--select",
            RUFF_RULES,
        ]
        if RUFF_IGNORE:
            ruff_cmd.extend(["--ignore", RUFF_IGNORE])
        ruff_cmd.append("-")

        result = subprocess.run(
            ruff_cmd, input=code, capture_output=True, text=True, check=False
        )
        if result.stdout:
            ruff_data = json.loads(result.stdout)
            for item in ruff_data:
                err_code = item["code"]
                # Skip import sorting, unused items, and stylistic issues from being blocking
                # I: Isort, F401: Unused import, F841: Unused variable, RUF005: Stylistic unpacking
                if err_code.startswith("I") or err_code in ["F401", "F841", "RUF005"]:
                    continue

                msg_text = item["message"]
                line_idx = item["location"]["row"]
                msg = f"[Ruff {err_code}] {msg_text} at line {line_idx}"
                errors.append(msg)
    except Exception as e:
        errors.append(f"[Linter Error] Ruff failed to run: {e}")

    if len(errors) >= MAX_ERRORS:
        return errors[:MAX_ERRORS]

    # 2. Run Pyrefly via temporary file
    with tempfile.NamedTemporaryFile(suffix=".py", mode="w", delete=False) as tmp:
        tmp.write(code)
        tmp_path = Path(tmp.name)

    try:
        pyrefly_cmd = ["pyrefly", "check", "--output-format", "json", str(tmp_path)]
        result = subprocess.run(
            pyrefly_cmd, capture_output=True, text=True, check=False
        )
        if result.stdout:
            raw_stdout = result.stdout.strip()
            if "{" in raw_stdout:
                start_idx = raw_stdout.find("{")
                end_idx = raw_stdout.rfind("}") + 1
                json_part = raw_stdout[start_idx:end_idx]
                pyrefly_data = json.loads(json_part)
                for item in pyrefly_data.get("diagnostics", []):
                    if IGNORE_WARNINGS and item.get("severity") == "warning":
                        continue

                    name = item.get("name", "type-error")
                    desc = item.get("concise_description") or item.get("description")
                    line = item.get("line", "?")
                    errors.append(f"[Pyrefly {name}] {desc} at line {line}")
    except Exception:
        pass
    finally:
        if tmp_path.exists():
            tmp_path.unlink()

    return errors[:MAX_ERRORS]
