import os
import re
import time
import uuid
from pathlib import Path

import httpx
import pytest
import yaml
from playwright.sync_api import Page, expect

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import AgentName, EpisodeStatus, TraceType
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    BenchmarkDefinition,
    BenchmarkPartDefinition,
    BenchmarkPartMetadata,
    BoundingBox,
    Constraints,
    CostTotals,
    MovedObject,
    ObjectivesSection,
    RandomizationMeta,
    StaticRandomization,
)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


def _write_workspace_file(
    client: httpx.Client,
    *,
    session_id: str,
    path: str,
    content: str,
    bypass_agent_permissions: bool = False,
) -> None:
    headers = {"X-Session-ID": session_id}
    if bypass_agent_permissions:
        headers["X-System-FS-Bypass"] = "1"

    resp = client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json={
            "path": path,
            "content": content,
            "overwrite": True,
            "bypass_agent_permissions": bypass_agent_permissions,
        },
        headers=headers,
        timeout=30.0,
    )
    assert resp.status_code == 200, resp.text


def _seed_engineer_handoff(client: httpx.Client, *, session_id: str) -> None:
    benchmark_definition = BenchmarkDefinition(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(-2.0, -2.0, 0.0), max=(2.0, 2.0, 10.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        benchmark_parts=[
            BenchmarkPartDefinition(
                part_id="environment_fixture",
                label="environment_fixture",
                metadata=BenchmarkPartMetadata(
                    material_id="aluminum_6061",
                ),
            )
        ],
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        payload=MovedObject(
            label="target_box",
            shape="sphere",
            material_id="aluminum_6061",
            static_randomization=StaticRandomization(radius=(1.0, 1.0)),
            start_position=(0.0, 0.0, 4.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(
            estimated_solution_cost_usd=10.0,
            estimated_solution_weight_g=100.0,
            max_unit_cost=15.0,
            max_weight_g=150.0,
            target_quantity=1,
        ),
        randomization=RandomizationMeta(
            static_variation_id="int-160-reasoning",
            runtime_jitter_enabled=True,
        ),
    )
    assembly_definition = AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=15.0,
            benchmark_max_weight_g=150.0,
            planner_target_max_unit_cost_usd=12.0,
            planner_target_max_weight_g=120.0,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[],
        totals=CostTotals(
            estimated_unit_cost_usd=10.0,
            estimated_weight_g=100.0,
            estimate_confidence="medium",
        ),
    )
    manufacturing_config = Path(
        "worker_heavy/workbenches/manufacturing_config.yaml"
    ).read_text(encoding="utf-8")

    _write_workspace_file(
        client,
        session_id=session_id,
        path="benchmark_definition.yaml",
        content=yaml.safe_dump(
            benchmark_definition.model_dump(mode="json", by_alias=True),
            sort_keys=False,
        ),
        bypass_agent_permissions=True,
    )
    _write_workspace_file(
        client,
        session_id=session_id,
        path="benchmark_assembly_definition.yaml",
        content=yaml.safe_dump(
            assembly_definition.model_dump(mode="json", by_alias=True),
            sort_keys=False,
        ),
        bypass_agent_permissions=True,
    )
    _write_workspace_file(
        client,
        session_id=session_id,
        path="assembly_definition.yaml",
        content=yaml.safe_dump(
            assembly_definition.model_dump(mode="json", by_alias=True),
            sort_keys=False,
        ),
        bypass_agent_permissions=True,
    )
    _write_workspace_file(
        client,
        session_id=session_id,
        path="manufacturing_config.yaml",
        content=manufacturing_config,
        bypass_agent_permissions=True,
    )


def _start_engineer_episode_via_api(*, task: str, session_id: str) -> str:
    with httpx.Client(timeout=120.0) as client:
        _seed_engineer_handoff(client, session_id=session_id)
        request = AgentRunRequest(
            task=task,
            session_id=session_id,
            agent_name=AgentName.ENGINEER_CODER,
            start_node=AgentName.ENGINEER_PLANNER,
        )
        response = client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=request.model_dump(mode="json"),
        )
        assert response.status_code in (200, 202), response.text
        episode_id = str(AgentRunResponse.model_validate(response.json()).episode_id)

        deadline = time.time() + 120.0
        while time.time() < deadline:
            episode_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
            assert episode_resp.status_code == 200, episode_resp.text
            episode = EpisodeResponse.model_validate(episode_resp.json())
            reasoning_traces = [
                trace
                for trace in episode.traces or []
                if trace.trace_type == TraceType.LLM_END
                and trace.name
                and trace.content
            ]
            if reasoning_traces:
                return episode_id
            if episode.status in {
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
                EpisodeStatus.CANCELLED,
            }:
                raise AssertionError(
                    f"Episode {episode_id} terminated before reasoning traces were persisted"
                )
            time.sleep(0.5)

    raise AssertionError(f"Episode {episode_id} never persisted reasoning traces")


@pytest.mark.integration_frontend
def test_int_160_reasoning_default_hidden_and_expandable(page: Page):
    """
    INT-160: Reasoning traces default-hidden + expandable.
    """
    page.set_viewport_size({"width": 1280, "height": 720})

    unique_task = f"INT-160 reasoning visibility {uuid.uuid4()}"
    session_id = f"INT-160-{uuid.uuid4().hex[:8]}"
    episode_id = _start_engineer_episode_via_api(
        task=unique_task,
        session_id=session_id,
    )

    with httpx.Client(timeout=10.0) as client:
        episode_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert episode_resp.status_code == 200, episode_resp.text
        episode = EpisodeResponse.model_validate(episode_resp.json())

    reasoning_candidates = [
        trace
        for trace in episode.traces or []
        if trace.trace_type == TraceType.LLM_END and trace.name and trace.content
    ]
    assert reasoning_candidates, "Expected persisted reasoning traces for INT-160."

    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.evaluate(
        "(episodeId) => localStorage.setItem('selectedEpisodeId', episodeId)",
        episode_id,
    )
    page.reload()

    page.wait_for_function(
        """(expectedEpisodeId) => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return data.episodeId === expectedEpisodeId;
            } catch (e) { return false; }
        }""",
        arg=episode_id,
        timeout=60000,
    )

    expect(page.get_by_test_id("view-reasoning-toggle")).to_have_text(
        re.compile(r"Off")
    )
    expect(page.locator('[data-testid="reasoning-span"]')).to_have_count(0)

    page.get_by_test_id("view-reasoning-toggle").click()
    expect(page.get_by_test_id("view-reasoning-toggle")).to_have_text(re.compile(r"On"))

    reasoning_spans = page.locator('[data-testid="reasoning-span"]')
    expect(reasoning_spans.first).to_be_visible(timeout=15000)
    reasoning_spans.first.click()
    expect(
        page.get_by_text(re.compile(r"Reasoning|Thought", re.IGNORECASE)).first
    ).to_be_visible()
