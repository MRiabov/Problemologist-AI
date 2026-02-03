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


def run_linter(code: str) -> list[dict]:
    """Runs ruff and pyrefly on the provided code and returns structured error data."""
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
            "--config",
            "builtins = ['show_object']",
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
                # I: Isort, F401: Unused import, F841: Unused variable,
                # RUF005: Stylistic unpacking, SIM117: Nested with, E501: Line length
                ignored_codes = ["F401", "F841", "RUF005", "SIM117", "E501"]
                if err_code.startswith("I") or err_code in ignored_codes:
                    continue

                # Special case: allow show_object as a global

                if err_code == "F821" and "show_object" in item["message"]:
                    continue

                errors.append(
                    {
                        "linter": "ruff",
                        "line": item["location"]["row"],
                        "code": err_code,
                        "message": item["message"],
                    }
                )
    except Exception as e:
        errors.append(
            {
                "linter": "linter-system",
                "line": 0,
                "code": "ERR",
                "message": f"Ruff failed to run: {e}",
            }
        )

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

                    # Skip undefined variable errors for show_object in Pyrefly
                    if "show_object" in desc and (
                        "undefined" in desc.lower() or "not found" in desc.lower()
                    ):
                        continue

                    line = item.get("line", "?")

                    errors.append(
                        {
                            "linter": "pyrefly",
                            "line": line,
                            "code": name,
                            "message": desc,
                        }
                    )
    except Exception as e:
        errors.append(
            {
                "linter": "pyrefly",
                "line": 0,
                "code": "ERR",
                "message": f"Pyrefly failed to run: {e}",
            }
        )
    finally:
        if tmp_path.exists():
            tmp_path.unlink()

    return errors[:MAX_ERRORS]


def format_linter_report(errors: list[dict]) -> str:
    """Formats the list of linter errors into a markdown report."""
    if not errors:
        return ""

    report_lines = ["Linter Feedback:", ""]

    for err in errors:
        line_info = f"Line {err['line']}" if err["line"] != "?" else "Unknown line"
        report_lines.append(
            f"{line_info}: {err['linter']} says: {err['message']} ({err['code']})"
        )
        report_lines.append("")

    # Add a generic suggested action for now, or could be more specific based on codes
    report_lines.append(
        "Suggested Action: Review the indicated lines and fix the reported issues. "
        "Ensure all variables are defined and types are consistent."
    )

    return "\n".join(report_lines)
