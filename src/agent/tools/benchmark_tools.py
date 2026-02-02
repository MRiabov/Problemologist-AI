import traceback
from langchain_core.tools import tool
from src.generators.benchmark.linter import run_linter, format_linter_report


@tool
def run_benchmark_linter(code: str) -> str:
    """
    Runs static analysis (Ruff/Pyrefly) on the provided code to check for syntax errors and undefined names.
    Use this before running validation to catch early errors.

    Args:
        code: The full python script content to check.
    """
    try:
        issues = run_linter(code)
        if not issues:
            return "Linting passed. No issues found."
        return format_linter_report(issues)
    except Exception as e:
        return f"Linter failed internally: {str(e)}"


@tool
def validate_benchmark_model(code: str, seed: int = 0) -> str:
    """
    Executes the build123d code to generate the MJCF environment and validates its structure.
    Returns the validation report or the generated MJCF if successful.

    Args:
        code: The full python script to execute.
        seed: Random seed for generation (default: 0).
    """
    # Import locally to avoid circular dependency (agent -> tools -> manager -> agent)
    from src.generators.benchmark.manager import execute_build
    from src.generators.benchmark.validator import validate_mjcf

    try:
        mjcf_xml = execute_build(code, seed=seed)
        report = validate_mjcf(mjcf_xml)

        if report["is_valid"]:
            # We return markers so the dashboard can extract the full MJCF even if it's large.
            # We still provide a truncated preview for the LLM to avoid context bloat.
            return (
                f"Validation Passed!\n"
                f"---FULL_MJCF_START---\n{mjcf_xml}\n---FULL_MJCF_END---\n"
                f"MJCF Preview:\n{mjcf_xml[:500]}..."
            )

        return f"Validation Failed:\n{report['error_message']}"
    except Exception:
        return f"Execution failed:\n{traceback.format_exc()}"
