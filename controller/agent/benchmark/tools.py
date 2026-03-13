import re
from collections.abc import Callable

import yaml

from controller.agent.tools import (
    _invoke_cots_search_subagent,
    filter_tools_for_agent,
    get_common_tools,
)
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.enums import AgentName
from shared.models.schemas import PlannerSubmissionResult


def get_benchmark_planner_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Planner-only filesystem toolset to keep planning loops focused."""

    def _normalized_tokens(value: str) -> tuple[str, ...]:
        return tuple(token for token in re.split(r"[^a-z0-9]+", value.lower()) if token)

    def _contiguous_phrases(tokens: tuple[str, ...]) -> set[tuple[str, ...]]:
        phrases: set[tuple[str, ...]] = set()
        for start in range(len(tokens)):
            for end in range(start + 1, len(tokens) + 1):
                phrases.add(tokens[start:end])
        return phrases

    def _phrase_matches(phrase_windows: set[tuple[str, ...]], candidate: str) -> bool:
        candidate_tokens = _normalized_tokens(candidate)
        return bool(candidate_tokens) and candidate_tokens in phrase_windows

    async def _benchmark_owned_cots_query_reason(query: str) -> str | None:
        if not await fs.exists("benchmark_definition.yaml"):
            return None

        try:
            raw = await fs.read_file("benchmark_definition.yaml")
            data = yaml.safe_load(raw) or {}
        except Exception:
            return None

        query_tokens = _normalized_tokens(query)
        if not query_tokens:
            return None
        query_phrases = _contiguous_phrases(query_tokens)

        benchmark_terms: set[str] = set()
        benchmark_parts = data.get("benchmark_parts")
        if isinstance(benchmark_parts, list):
            for part in benchmark_parts:
                if not isinstance(part, dict):
                    continue
                for field_name in ("part_id", "label"):
                    field_value = part.get(field_name)
                    if isinstance(field_value, str) and field_value.strip():
                        benchmark_terms.add(field_value)

        matches = sorted(
            term for term in benchmark_terms if _phrase_matches(query_phrases, term)
        )
        if not matches:
            return None

        return (
            "Benchmark-owned fixtures are read-only task context and are excluded "
            "from COTS pricing/manufacturability. "
            f"Do not use invoke_cots_search_subagent for query {query!r}. "
            f"Matched benchmark-owned term(s): {', '.join(matches)}. "
            "Use one heuristic estimate for the likely engineer-side solution instead."
        )

    async def list_files(path: str = "/"):
        return await fs.list_files(path)

    async def read_file(path: str):
        return await fs.read_file(path)

    async def write_file(path: str, content: str, overwrite: bool = False):
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit_file(path: str, old_string: str, new_string: str):
        from controller.middleware.remote_fs import EditOp

        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def grep(pattern: str, path: str | None = None, glob: str | None = None):
        return await fs.grep(pattern, path, glob)

    async def invoke_cots_search_subagent(
        query: str,
        max_weight_g: float | None = None,
        max_cost: float | None = None,
        category: str | None = None,
        limit: int = 5,
    ) -> str:
        """
        Invoke the dedicated COTS search subagent for likely engineer-side parts only.

        Benchmark-owned fixtures must not be priced through catalog search.
        """
        blocked_reason = await _benchmark_owned_cots_query_reason(query)
        if blocked_reason:
            raise ValueError(blocked_reason)
        return await _invoke_cots_search_subagent(
            query=query,
            max_weight_g=max_weight_g,
            max_cost=max_cost,
            category=category,
            limit=limit,
            session_id=session_id,
        )

    async def submit_plan() -> dict:
        """
        Validate benchmark planner artifacts and explicitly submit planner handoff.

        Returns:
            {"ok": bool, "status": "submitted"|"rejected", "errors": [...], "node_type": "..."}
        """
        from worker_heavy.utils.file_validation import validate_node_output

        required_files = ["plan.md", "todo.md", "benchmark_definition.yaml"]
        artifacts: dict[str, str] = {}
        missing_files: list[str] = []

        for rel_path in required_files:
            if not await fs.exists(rel_path):
                missing_files.append(rel_path)
                continue
            content = await fs.read_file(rel_path)
            if not content.strip():
                missing_files.append(rel_path)
                continue
            artifacts[rel_path] = content

        if missing_files:
            result = PlannerSubmissionResult(
                ok=False,
                status="rejected",
                errors=[f"Missing required file: {p}" for p in missing_files],
                node_type=AgentName.BENCHMARK_PLANNER,
            )
            return result.model_dump(mode="json")

        is_valid, errors = validate_node_output(AgentName.BENCHMARK_PLANNER, artifacts)
        result = PlannerSubmissionResult(
            ok=is_valid,
            status="submitted" if is_valid else "rejected",
            errors=errors,
            node_type=AgentName.BENCHMARK_PLANNER,
        )
        return result.model_dump(mode="json")

    return filter_tools_for_agent(
        fs,
        [
            list_files,
            read_file,
            write_file,
            edit_file,
            grep,
            invoke_cots_search_subagent,
            submit_plan,
        ],
    )


def get_benchmark_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    # Benchmark ReAct tools intentionally exclude validate/simulate:
    # these are Python utils imported and called from script context.
    return get_common_tools(fs, session_id)
