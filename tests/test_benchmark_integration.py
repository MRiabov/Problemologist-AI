import pytest
from langchain_core.messages import HumanMessage
from src.generators.benchmark.agent import generator_agent, DEFAULT_RUNTIME_CONFIG


@pytest.mark.benchmark
@pytest.mark.integration
@pytest.mark.asyncio
async def test_benchmark_generation_integration():
    """
    Integration test that uses a real LLM to generate a benchmark from a plan.
    """

    plan_content = """
### 1. Learning Objective
This benchmark tests **3D spatial routing and constrained geometry design**. The CAD agent must generate a manifold part that acts as a bridge/ramp.
*   **Specific CAD Capability**: Pathfinding around obstacles, maintaining a constant downward slope (functional gradient), and cross-sectional design (creating a track/groove to constrain a sphere).

### 2. Static Geometry
The environment consists of three fixed elements within a 100x100x100mm domain:
*   **Start Platform**: A small 15x15mm horizontal ledge located at `[x=-40, y=40, z=90]`. This is where the ball is spawned.
*   **The Obstacle**: A vertical pillar (cylinder or box) centered at `[x=0, y=0]` extending from `z=0` to `z=70`. This blocks the direct linear path from start to goal.
*   **The Goal Bin**: A shallow open-topped box at the floor level `[z=0]`, centered at `[x=40, y=-40]`. 
*   **Mounting Points**: Two small threaded-hole analogs on the Start Platform and the Goal Bin where the "Solution" part is expected to interface.

### 3. Interactive Elements
*   **The Projectile**: A 10mm diameter sphere (mass: 0.05kg) with low rolling friction.
*   **Dynamics**: The ball is spawned 5mm above the Start Platform. It falls onto the platform (or the agent's part) and must roll via gravity to the goal.

### 4. Success Criteria
*   **Win Condition**: The ball's center of mass must enter an Axis-Aligned Bounding Box (AABB) defined by the Goal Bin: `[x: 30 to 50, y: -50 to -30, z: 0 to 20]`.
*   **Stability**: The ball must remain within the bounds of the agent's designed part throughout the transit (no flying off the track).
*   **Time Limit**: The ball must reach the goal within 5 seconds of simulation time.

### 5. Rescale Limits
*   **X-axis**: 0.8 to 1.2 (Adjusts the width of the workspace).
*   **Y-axis**: 0.8 to 1.2 (Adjusts the depth of the workspace).
*   **Z-axis**: 1.0 (Height remains fixed to maintain gravity constants).

### 6. Randomization
To ensure the agent uses parametric logic rather than hardcoded coordinates:
*   **Obstacle Position**: The center of the pillar shifts by `±10mm` in X and Y.
*   **Obstable Height**: The pillar height varies between `60mm` and `80mm`.
*   **Goal Position**: The Goal Bin rotates around the center of the workspace by `±15 degrees`.
*   **Ball Diameter**: The ball size varies between `8mm` and `12mm` (requiring the agent to adjust the width of their track/chute).
"""

    # New AgentState structure
    state = {
        "messages": [
            HumanMessage(
                content="A ball drop puzzle with a pillar obstacle and a goal bin."
            )
        ],
        "plan": plan_content,
        "runtime_config": DEFAULT_RUNTIME_CONFIG,
        "step_count": 0,
        "scratchpad": {},
    }

    result = await generator_agent.ainvoke(state, config={"recursion_limit": 50})

    # Check for success in valid messages
    mjcf = None
    validation_passed = False

    for msg in reversed(result["messages"]):
        content = str(msg.content)
        if "Validation Passed!" in content and "MJCF Output" in content:
            validation_passed = True
            # Extract basic check (or parse if needed)
            if "<mujoco" in content:
                mjcf = content
            break

    if not validation_passed:
        print("\nINTEGRATION TEST FAILED")
        print(f"Total Steps: {len(result['messages'])}")
        print("\nDEBUG: Integration Test Messages Trace:")
        for m in result["messages"]:
            print(
                f"[{type(m).__name__}]: {m.content[:100]}... | ID: {getattr(m, 'id', 'N/A')} | TC: {getattr(m, 'tool_calls', 'N/A')}"
            )
        # Print last few messages
        for m in result["messages"][-5:]:
            print(f"[{type(m).__name__}]: {m.content[:500]}...")

    trace = "\n".join(
        [
            f"[{type(m).__name__}]: {m.content[:100]}... | TC: {getattr(m, 'tool_calls', 'N/A')}"
            for m in result["messages"]
        ]
    )
    assert validation_passed is True, f"Benchmark generation failed. Trace:\n{trace}"
    assert mjcf is not None
