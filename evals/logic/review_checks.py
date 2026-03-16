import re

import yaml
from pydantic import ValidationError

from controller.clients.worker import WorkerClient
from evals.logic.models import AgentEvalSpec, EvalDatasetItem
from shared.models.schemas import ReviewFrontmatter
from worker_heavy.utils.file_validation import validate_review_frontmatter


def requires_expected_review_decision(spec: AgentEvalSpec) -> bool:
    return spec.review_filename_prefix is not None


def review_artifact_pending(error: str | None) -> bool:
    if not error:
        return False
    return error in {
        "reviews/ directory not found",
        "missing persisted review frontmatter",
    } or error.startswith("no persisted review artifact matching ")


def review_filename_candidates(review_filename_prefix: str) -> list[str]:
    candidates = [review_filename_prefix]
    if review_filename_prefix.endswith("-round"):
        candidates.append(review_filename_prefix[: -len("-round")])
    return list(dict.fromkeys(candidates))


def parse_review_decision_yaml(
    content: str, *, review_path: str
) -> tuple[ReviewFrontmatter | None, str | None]:
    try:
        data = yaml.safe_load(content)
    except yaml.YAMLError as exc:
        return None, f"invalid YAML in {review_path}: {exc}"

    if not isinstance(data, dict):
        return None, f"unexpected YAML structure in {review_path}: expected mapping"

    try:
        return ReviewFrontmatter.model_validate(data), None
    except ValidationError as exc:
        errors = ", ".join(
            f"{'.'.join(str(part) for part in err['loc'])}: {err['msg']}"
            for err in exc.errors()
        )
        return None, f"invalid review decision payload in {review_path}: {errors}"


async def load_latest_review_frontmatter(
    *,
    worker: WorkerClient,
    review_filename_prefix: str,
    session_id: str,
) -> tuple[ReviewFrontmatter | None, str | None]:
    try:
        entries = await worker.list_files("reviews", bypass_agent_permissions=True)
    except FileNotFoundError:
        return None, "reviews/ directory not found"
    except Exception as exc:
        return None, f"failed to list reviews/: {exc}"

    prefixes = review_filename_candidates(review_filename_prefix)
    yaml_patterns = [
        re.compile(rf"^{re.escape(prefix)}-decision-round-(\d+)\\.yaml$")
        for prefix in prefixes
    ]
    md_patterns = [
        re.compile(rf"^{re.escape(prefix)}-(\d+)\\.md$") for prefix in prefixes
    ]

    latest_yaml_name: str | None = None
    latest_yaml_round = -1
    latest_md_name: str | None = None
    latest_md_round = -1
    for entry in entries:
        if entry.is_dir:
            continue
        for pattern in yaml_patterns:
            match = pattern.fullmatch(entry.name)
            if match is not None:
                round_number = int(match.group(1))
                if round_number > latest_yaml_round:
                    latest_yaml_round = round_number
                    latest_yaml_name = entry.name
                break
        else:
            for pattern in md_patterns:
                match = pattern.fullmatch(entry.name)
                if match is not None:
                    round_number = int(match.group(1))
                    if round_number > latest_md_round:
                        latest_md_round = round_number
                        latest_md_name = entry.name
                    break

    if latest_yaml_name is not None:
        review_path = f"reviews/{latest_yaml_name}"
        try:
            content = await worker.read_file(review_path, bypass_agent_permissions=True)
        except Exception as exc:
            return None, f"failed to read {review_path}: {exc}"

        review_data, review_error = parse_review_decision_yaml(
            content,
            review_path=review_path,
        )
        if review_error is not None:
            return None, review_error
        if review_data is None:
            return None, f"missing review decision payload in {review_path}"
        return review_data, None

    if latest_md_name is not None:
        review_path = f"reviews/{latest_md_name}"
        try:
            content = await worker.read_file(review_path, bypass_agent_permissions=True)
        except Exception as exc:
            return None, f"failed to read {review_path}: {exc}"

        is_valid, review_data = validate_review_frontmatter(
            content,
            cad_agent_refused=True,
            session_id=session_id,
        )
        if not is_valid:
            details = (
                ", ".join(review_data)
                if isinstance(review_data, list)
                else str(review_data)
            )
            return None, f"invalid review frontmatter in {review_path}: {details}"
        if not isinstance(review_data, ReviewFrontmatter):
            return None, f"unexpected review frontmatter payload for {review_path}"
        return review_data, None

    return (
        None,
        "no persisted review artifact matching "
        f"reviews/{review_filename_prefix}-decision-round-<n>.yaml "
        f"or reviews/{review_filename_prefix}-<n>.md",
    )


async def review_artifacts_complete_for_prefix(
    *,
    worker: WorkerClient,
    review_filename_prefix: str,
) -> bool:
    try:
        entries = await worker.list_files("reviews", bypass_agent_permissions=True)
    except Exception:
        return False

    prefixes = review_filename_candidates(review_filename_prefix)
    decision_patterns = [
        (prefix, re.compile(rf"^{re.escape(prefix)}-decision-round-(\d+)\\.yaml$"))
        for prefix in prefixes
    ]

    latest_prefix: str | None = None
    latest_round = -1
    for entry in entries:
        if entry.is_dir:
            continue
        for prefix, pattern in decision_patterns:
            match = pattern.fullmatch(entry.name)
            if match is None:
                continue
            round_number = int(match.group(1))
            if round_number > latest_round:
                latest_round = round_number
                latest_prefix = prefix
            break

    if latest_prefix is not None and latest_round > 0:
        expected_comments = f"{latest_prefix}-comments-round-{latest_round}.yaml"
        return any(
            not entry.is_dir and entry.name == expected_comments for entry in entries
        )

    md_patterns = [
        re.compile(rf"^{re.escape(prefix)}-(\d+)\\.md$") for prefix in prefixes
    ]
    return any(
        not entry.is_dir
        and any(pattern.fullmatch(entry.name) for pattern in md_patterns)
        for entry in entries
    )


async def planner_artifacts_present(
    *,
    worker: WorkerClient,
    required_files: tuple[str, ...],
) -> bool:
    if not required_files:
        return False

    for rel_path in required_files:
        try:
            if not await worker.exists(rel_path, bypass_agent_permissions=True):
                return False
            content = await worker.read_file(rel_path, bypass_agent_permissions=True)
        except Exception:
            return False
        if not isinstance(content, str) or not content.strip():
            return False
    return True


async def review_expectation_error(
    *,
    item: EvalDatasetItem,
    spec: AgentEvalSpec,
    session_id: str,
    worker_light_url: str,
) -> str | None:
    expected_decision = item.expected_decision
    review_filename_prefix = spec.review_filename_prefix
    if expected_decision is None or review_filename_prefix is None:
        return None

    worker = WorkerClient(base_url=worker_light_url, session_id=session_id)
    try:
        review_frontmatter, review_error = await load_latest_review_frontmatter(
            worker=worker,
            review_filename_prefix=review_filename_prefix,
            session_id=session_id,
        )
    finally:
        await worker.aclose()

    if review_error:
        return review_error
    if review_frontmatter is None:
        return "missing persisted review frontmatter"
    if review_frontmatter.decision != expected_decision:
        return (
            "expected review decision "
            f"{expected_decision.value}, got {review_frontmatter.decision.value}"
        )
    return None
