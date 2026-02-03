# Requirements Quality Checklist: Local Environment Tools (001)

**Feature**: 001-agentic-cad-environment
**Status**: Foundation
**Purpose**: This checklist validates the quality, clarity, and completeness of the requirements for the local agentic CAD tools and data persistence.

## Requirement Completeness

- [ ] CHK001 - Are requirements defined for handling script execution timeouts in the `preview_design` tool? [Gap]
  - **Proposed Change:** Enforce a 30-second hard timeout. If exceeded, terminate process and return `TimeoutError` observation to agent.
  - **User Review:**
  I think we have a timeout after 30-60 seconds in the codebase. But yes, we need it.

- [ ] CHK003 - Are the data source boundaries and versioning requirements specified for the `search_docs` (RAG) tool? [Completeness, Spec §4.2]
  - **Proposed Change:** Source restricted to bundled `build123d` and `numpy` docs. Version must match the installed pip package versions.
  - **User Review:**

- [ ] CHK004 - Are error handling requirements defined for malformed Python scripts in the `preview_design` flow? [Gap, Spec §4.2]
  - **Proposed Change:** Capture `SyntaxError` at runtime. Return formatted traceback as text observation. Do not crash the environment.
  - **User Review:**

- [ ] CHK005 - Does the spec define requirements for handling concurrent file access if multiple tools try to modify the same script? [Gap]
  - **Proposed Change:** Assume single-threaded agent execution. If OS-level lock encountered, return `FileBusyError` and retry hint.
  - **User Review:**

## Requirement Clarity

- [ ] CHK006 - Is the format of the "visual snapshot" (e.g., resolution, file type, view angle) explicitly specified for `preview_design`? [Clarity, Spec §2.2]
  - **Proposed Change:** RGB PNG, 1024x1024px, fixed isometric view (45, 45, 35) zoom-to-fit.
  - **User Review:**

- [ ] CHK007 - Is the "2 second" pipeline latency defined as a hard constraint or a target goal? [Ambiguity, Spec §2.2]
  - **Proposed Change:** Target goal (P95). If >5s, log warning but do not fail the step.
  - **User Review:**

- [ ] CHK008 - Are the "specific error messages" (tracebacks vs. geometric violations) quantified with examples of required fields? [Clarity, Spec §3]
  - **Proposed Change:** Required fields: `error_type` (e.g., GeometryError, SyntaxError), `message`, `line_number`, and `traceback`.
  - **User Review:**

- [ ] CHK009 - Is "intrinsic visual feedback" defined with measurable success criteria for the agent's observation? [Ambiguity, Spec §4.1]
  - **Proposed Change:** Success = Agent receives valid file path to a non-empty PNG and basic geometry stats (e.g., bounding box volume > 0).
  - **User Review:**

## Requirement Consistency

- [ ] CHK010 - Do the observation requirements in §4.1 align with the tool outputs defined in §4.2? [Consistency]
  - **Proposed Change:** Yes. Tool output structure is the single source of truth for observation format.
  - **User Review:**

- [ ] CHK011 - Are the requirements for `write_script` consistent regarding file path validation? [Consistency]
  - **Proposed Change:** Yes. Must enforce sandbox confinement (reject `..` path traversal) and mandate relative paths.
  - **User Review:**

## Data Strategy & Persistence

- [ ] CHK012 - Are the requirements for capturing "Thought-Process" data (e.g., intermediate tool calls, reasoning steps) explicitly defined in the schema? [Gap, Spec §2.1/4.5]
  - **Proposed Change:** Add `thought_trace` column to Steps table to store raw CoT text if provided by the model.
  - **User Review:**

- [ ] CHK013 - Does the spec define requirements for deduplication or versioning of Python scripts stored in the `Artifacts` table? [Gap, Spec §4.5]
  - **Proposed Change:** Store every script execution as a new immutable row linked to `step_id`. No deduplication for traceability.
  - **User Review:**

- [ ] CHK014 - Are the exact metadata fields (e.g., execution time, return codes, stack traces) for the `Steps` table specified? [Completeness, Spec §4.5]
  - **Proposed Change:** Schema: `step_id`, `timestamp`, `duration_ms`, `exit_code`, `cli_output` (stdout/stderr combined), `error_trace`.
  - **User Review:**

- [ ] CHK015 - Is there a requirement for logging the state of the CAD kernel (e.g., active objects, memory usage) during a `preview_design` call? [Gap]
  - **Proposed Change:** Log simple `object_count` and `peak_memory_mb` in step metadata.
  - **User Review:**

## Scenario & Edge Case Coverage

- [ ] CHK016 - Are requirements defined for the environment's behavior when a script creates infinite loops or consumes excessive memory? [Gap]
  - **Proposed Change:** 1GB RAM hard limit per script. Kill process on violation. (See CHK001 for timeout).
  - **User Review:**

- [ ] CHK017 - Does the spec define requirements for "zero-state" observations (e.g., first step of an episode)? [Coverage]
  - **Proposed Change:** Initial observation: "Workspace empty. Available tools: ...". No script exists yet.
  - **User Review:**

- [ ] CHK018 - Are requirements specified for handling failures in the RAG retrieval system (e.g., no snippets found)? [Coverage, Spec §4.2]
  - **Proposed Change:** Return explicit observation: "No relevant documentation found for: [query]". Do not hallucinate results.
  - **User Review:**

- [ ] CHK019 - Does the spec define behavior for when a `preview_design` call is made before any script has been written? [Edge Case]
  - **Proposed Change:** Return user error: "FileNotFound: [script_path] does not exist. Please create it first."
  - **User Review:**

## Traceability & Measurability

- [ ] CHK020 - Can the "Data Integrity" success criterion be objectively verified with specific validation rules? [Measurability, Spec §2.2]
  - **Proposed Change:** Rule: `hash(file_on_disk) == hash(db_artifact_blob)` for every step.
  - **User Review:**

- [ ] CHK021 - Is there a traceability requirement linking tool failures to specific logged error states in SQLite? [Traceability]
  - **Proposed Change:** Yes. A failing tool call must produce a Step row with `status='FAILED'` and populated `error_trace`.

## Architectural Constraints & Security

- [ ] CHK022 - Is there a strict requirement banning host-side code execution? [Security]
  - **Proposed Change:** All user-provided code and tool-generated scripts must execute inside the Podman sandbox. No `exec()` on host.
  - **User Review:**

- [ ] CHK023 - Are workspace isolation requirements defined for parallel testing? [Architecture]
  - **Proposed Change:** Each test/run must use a unique workspace path (e.g., UUID-based). Shared headers/folders are forbidden.
  - **User Review:**

- [ ] CHK024 - Are structured outputs required for all tools? [Architecture]
  - **Proposed Change:** Tools must return Pydantic models or Dataclasses, converted to JSON for the agent. No raw dicts.
  - **User Review:**

- [ ] CHK025 - Is the dependency on `gym`/`gymnasium` explicitly forbidden? [Architecture]
  - **Proposed Change:** The Environment interface must be purely functional/service-based. No `gym` inheritance or action spaces.
  - **User Review:**
