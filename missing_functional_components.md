# Missing Functional Components Report

This report identifies 12 missing pieces of functionality that are specified in `specs/desired_architecture.md` but are currently absent from the implementation.

---

### 1. Electrical Reviewer Node
- **Architectural Reference**: "The Engineer consists of... Electrical reviewer" (Line 565); "Lead engineer <-> CAD modelling engineer <-> Engineering Reviewer" (Line 900).
- **Description**: The architecture specifies a dedicated Electrical Reviewer node within the engineering graph to validate circuit designs and electrical integration.
- **Evidence of Absence**: `controller/agent/graph.py` defines `plan_reviewer` and `execution_reviewer` (Mechanical), but lacks an `electronics_reviewer`. No implementation exists in `controller/agent/nodes/`.

### 2. Token Compression and Summarization
- **Architectural Reference**: "Token compression" (Line 773); "As the agent will near its token generation limits, they will compress their old memory by a summarizing agent." (Lines 773-775).
- **Description**: A mechanism to summarize history and compress the context window when approaching LLM limits is missing.
- **Evidence of Absence**: `controller/agent/nodes/base.py` and `controller/agent/state.py` do not implement any summarization or windowing logic based on token counts.

### 3. Skills Agent Non-Duplication Check
- **Architectural Reference**: "Non-duplication: Generated skills do not duplicate existing skills (upon inspecting git changes after 30 turns- the skill rows aren't churned)" (Lines 1426-1433).
- **Description**: The skills sidecar should verify that it isn't creating skills that already exist or causing "churn" in the git repository.
- **Evidence of Absence**: `SkillsNode` in `controller/agent/nodes/skills.py` only implements a "deletion safety toggle" (lines 64-88) but does not perform any semantic or structural duplication checks.

### 4. Automated Git Commits for Simulation Traceability
- **Architectural Reference**: "We want to track code evolution. Hence, we will use `git add . && git commit` during every successful `simulate` call." (Complexity tracking, Lines 2555+).
- **Description**: The system is intended to capture the "evolution" of the solution by committing to git whenever a simulation succeeds.
- **Evidence of Absence**: Neither the `simulate` endpoint in `worker_heavy/api/routes.py` nor the utility in `worker_heavy/utils/validation.py` includes git commit logic.

### 5. Benchmark CAD Agent Refusal Mechanism
- **Architectural Reference**: "If plan is not valid - e.g. it specifies a conflicting geometry, the CAD agent will refuse; and send back to benchmark planner agent." (Line 900).
- **Description**: The handover between the Benchmark Planner and Benchmark CAD Agent should allow for "refusal" if the plan's geometry is physically impossible or conflicting.
- **Evidence of Absence**: The `coder_node` in `controller/agent/benchmark/nodes.py` lacks a structured route back to the `planner` node for plan refinement; transitions in `controller/agent/benchmark/graph.py` only flow forward to the reviewer.

### 6. Reviewer Inspection of COTS Search Queries
- **Architectural Reference**: "Reviewers may search for a better component (suggestion: reviewers may want to read the search queries of invoking agents to decide if the part found was sufficiently good or not.)" (Line 538).
- **Description**: Reviewers should have access to the search history and queries used by the planner/engineer to evaluate the quality of component selection.
- **Evidence of Absence**: Reviewer nodes currently only receive the final `plan.md` or `assembly_definition.yaml` and do not receive the `cots_search` history or logs.

### 7. Dynamic Camera Rendering based on Geometric Selection
- **Architectural Reference**: "If a user selects a particular feature, the system automatically determines the best angle... and render the image with this view selected part more brightly." (Lines 1250+).
- **Description**: Steerability logic requires the system to render specific views based on user-selected faces or parts to provide visual context to the agent.
- **Evidence of Absence**: `worker_heavy/utils/preview.py` and the `preview_design` tool use static or user-defined pitch/yaw and do not implement automatic "best angle" determination for specific target IDs.

### 8. Automated "Complaining" Agent (Debugging Sidecar)
- **Architectural Reference**: "A LLM that explicitly checks where the model got confused... The developer can use the LLM output to debug." (Lines 2160-2170).
- **Description**: An asynchronous sidecar model dedicated to identifying and reporting developer-facing bugs or "confusion points" in reasoning traces.
- **Evidence of Absence**: While a `SkillsNode` exists for learning, there is no dedicated "debugger" sidecar that populates a developer issues database as described.

### 9. Skill Safety Timeout Reversion
- **Architectural Reference**: "If the session times out (too many tool calls/the agent finishes despite warnings), we revert the commits." (Lines 863-869).
- **Description**: If the skill creator agent exceeds its limits or times out after modifying files, the system must automatically revert the git commits to prevent corrupted or oversized skill files.
- **Evidence of Absence**: `SkillsNode.py` implements a write-time block but lacks the state-aware logic to perform a git revert if the overall session fails or times out.

### 10. Benchmark Solvability Verification
- **Architectural Reference**: "Benchmark reviewer node: reviews the environment for... Feasibility of solution" (Line 257).
- **Description**: The benchmark reviewer is tasked with ensuring the generated challenge is actually solvable by an engineering agent.
- **Evidence of Absence**: The `BenchmarkReviewerNode` in `controller/agent/benchmark/nodes.py` checks for geometric validity and simulation stability but lacks a "solvability" check (e.g., via a mock solve or specific feasibility analysis).

### 11. DFM Suggestions in Planner Tools
- **Architectural Reference**: "Future: will also add some basic planning suggestions. e.g.: it appears you are trying to CNC away over 80% of the stock. Consider picking a planning to use a smaller stock if possible." (Lines 2292+).
- **Description**: The planning-phase validation tool (`validate_costing_and_price.py`) should provide Design for Manufacturability (DFM) hints to help the planner optimize.
- **Evidence of Absence**: The implementation in `.agent/skills/manufacturing-knowledge/scripts/validate_costing_and_price.py` is strictly a budget calculator and does not emit architectural or DFM suggestions.

### 12. Skills Merge Conflict Resolution via LLM
- **Architectural Reference**: "If push fails due to merge conflict, do git merge. If merge fails, have the skill creator LLM handle it and push it." (Lines 642+).
- **Description**: The skills creator should be capable of resolving git merge conflicts when pushing to the shared repository.
- **Evidence of Absence**: `GitManager` in `controller/utils/git.py` handles basic sync, but there is no logic in `SkillsNode` to detect a merge failure and prompt the LLM to resolve the resulting conflict markers.
