import pytest

from src.generators.benchmark.agent import generator_agent


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
*   **Obstacle Height**: The pillar height varies between `60mm` and `80mm`.
*   **Goal Position**: The Goal Bin rotates around the center of the workspace by `±15 degrees`.
*   **Ball Diameter**: The ball size varies between `8mm` and `12mm` (requiring the agent to adjust the width of their track/chute).
"""

    # We skip the planner stage by providing the plan directly in the state
    state = {
        "request": "A ball drop puzzle with a pillar obstacle and a goal bin.",
        "plan": plan_content,
        "attempts": 0,
        "validation_passed": False,
        "full_history": []
    }
    
    # We invoke the agent starting from the 'coder' node to use the provided plan
    # However, langgraph's compiled graph starts from the entry point.
    # To start from a specific state, we can use the graph as is, 
    # but we need to ensure the planner doesn't overwrite our plan if we skip it.
    
    # Let's try invoking the whole thing first, or see if we can jump nodes.
    # Since we want to test the WHOLE pipeline (including reasoning extraction),
    # let's just provide the request and let it plan, then we can verify if it follows it.
    
    # result = await generator_agent.ainvoke({"request": "Design a ball drop puzzle exactly as described in the requirements."}) 
    
    # To strictly test the plan provided by the user:
    # We can use the graph's internal nodes if needed, but ainvoke is better.
    # If we want to skip planner, we might need a slightly different graph or just a mock planner.
    
    # Actually, the user wants to test IF PASSING IN A PLAN it completes.
    # So we should simulate being in the PLAN_APPROVAL stage where plan is already set.
    
    result = await generator_agent.ainvoke(state)
    
    if not result["validation_passed"]:
        print("\nINTEGRATION TEST FAILED")
        print(f"Total Attempts: {result.get('attempts')}")
        
        history = result.get("full_history", [])
        print(f"--- Attempt History ({len(history)} items) ---")
        for i, entry in enumerate(history):
            print(f"Attempt {i+1} Result: {'VALID' if entry['is_valid'] else 'FAILED'}")
            if not entry['is_valid']:
                print(f"Error: {entry['error_message']}")
            print("-" * 20)

        if result.get("coder_reasoning"):
            print(f"Last Coder Reasoning: {result['coder_reasoning'][:500]}...")
        if result.get("code"):
            print(f"Last Generated Code Sample: {result['code'][:500]}...")

    assert result["validation_passed"] is True
    assert result["mjcf"] is not None
    assert "<mujoco" in result["mjcf"]
