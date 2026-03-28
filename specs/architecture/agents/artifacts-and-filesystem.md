# Agentic framework

## Scope summary

- Primary focus: runtime framework and filesystem contract for all agent workflows.
- Defines DSPy/LangGraph/Langfuse architecture and strict reasoning/tool trace requirements.
- Specifies agent artifact surfaces, shared template sources, and role-based path permissions (`agents_config.yaml` policy model).
- Use this file for runtime plumbing, file immutability, and access-control decisions.

We use DSPy.ReAct as the primary agent runtime (reasoning + tool use) and LangGraph to manage agent architecture/orchestration. Langfuse is used for observability.

## DSPy adapter contract

Adapter choice is an explicit runtime contract:

1. Preferred/default adapter is `dspy.ChatAdapter`.
2. We prefer `ChatAdapter` over `JSONAdapter` for ReAct-style agent nodes because it is the baseline behavior in our dependency/runtime and is the most stable path for mixed reasoning + tool-call trajectories.
3. `JSONAdapter` is exception-only and must be justified by a node-specific structured-output requirement that cannot be met by normal typed parsing/validation after model output.
4. Do not silently swap adapters per request. Adapter overrides must be explicit in code/config and visible in observability metadata.
5. Fail-closed rule: if adapter selection is invalid/unsupported for a node, the node must fail with a deterministic runtime error instead of falling back silently.

## Implementation

We create two LangGraph-managed workflows with DSPy.ReAct-driven agent nodes - a Benchmark Generator workflow, consisting of:

- Planner,
- Plan Reviewer,
- Coder,
- Reviewer,
- Skill Agent
- COTS Search subagent.

The Engineer consists of

- Planner,
- Electrical Planner
- Coder,
- Engineering Plan Reviewer,
- Engineering Execution Reviewer,
- Electronics Reviewer
- Skill Agent
- COTS Search subagent.

I decided to split the electrical planner because of complexity and different skills that may be involved in electrical engineering.

We trace all workflows with LangFuse.

### Actual implementation

<!--This is because models implementing this produced something else.-->

The pipeline is the simplest, proper way to call LLMs:
DSPy (LiteLLM) calls an endpoint, we get either reasoning, tool call, or both. We send it to frontend and to observability (and log it). that's all the intended logic.

Reasoning and tool observability are strict runtime contracts:

1. LM outputs are emitted as trace records incrementally during execution (streaming), not only at node end or episode end.
2. Tool call traces are emitted at call start with exact arguments, and updated with observation/error at call end.
3. Reasoning visibility in frontend is backed only by persisted backend traces for the same `episode_id`; frontend must not synthesize placeholder reasoning rows.
4. We do not fabricate reasoning traces from synthetic defaults or inferred placeholders. If no reasoning payload is produced by the LM/runtime, the run is marked as missing required telemetry.
5. For debugging/benchmark dataset modes, reasoning telemetry is fail-closed: a planner/execution stage that requires reasoning traces and produces none must not transition to success-like statuses.
6. `trace_type` values such as `LLM_END` are observability event labels, not workflow termination signals. Episode termination is controlled only by workflow outcomes (success/failure/timeout/cancel).
7. Reasoning traces should carry optional metadata `reasoning_step_index` and `reasoning_source` for deterministic ordering/provenance in frontend and evaluations.

#### Extracting reasoning

As an implementation detail, here is how we extract reasoning:

```py
from litellm import completion  
import os  
  
os.environ['OPENROUTER_API_KEY'] = ""  
resp = completion(  
 model="openrouter/provider-name/model-name",  
 messages=[{"role": "user", "content": "Tell me a joke."}],  
)  
  
print(resp.choices[0].message.reasoning_content)  
```

## Filesystem

Both of the agents "live" directly in the filesystem of the container that they have been assigned to and thus runs their workflow. This serves the purpose of reducing complexity in tooling, and giving the agents the familiarity with editing tools. There are skills, a script to be written, and verification tools in the script.

Agent access is sandbox-bound: an agent must not have read, write, or execution access to filesystem paths outside its assigned sandbox/workspace, except through explicitly mounted read-only runtime inputs that are part of that sandbox contract. Any escape to the host filesystem or repository root outside the sandbox is a contract violation.

`execute_command(...)` follows that same terminal model: it is intended to run shell commands from the session workspace root. If any current implementation path treats it as a raw-Python-only executor, that is implementation drift rather than the architecture contract.

Agent-authored Python scripts are expected to import runtime helpers from a top-level `utils` package. The submission script contract is:

```py
from utils.submission import validate, simulate, submit_for_review
from utils.metadata import PartMetadata, CompoundMetadata
```

The old `shared.*` import paths remain repo internals, not the submission script API contract.

## Templates

Reusable starter files are defined once in `shared/agent_templates/common/`. Controller initialization, worker startup, eval workspace materialization, and integration-test fixture expansion copy those files into each workspace before node entry. `worker_light/agent_files/` is the runtime mirror used by worker bootstrap and local inspection; it must not drift from `shared/agent_templates/common/`.

Row-specific seed artifacts still live in their row-local seed directories. The shared template pack covers only the boilerplate files that should not be duplicated across folders. Role-specific planner scaffolds remain in their existing template-repo locations and are not duplicated into the common starter pack.

### Initial files for each agent and read-write permissions

- Engineering Planner:
  - read: `skills/**`, `utils/**`, `benchmark_definition.yaml` (benchmark-owned constraints), `plan.md` (if materialized from shared templates), `todo.md` (if materialized from shared templates), `journal.md` (if materialized from shared templates)
  - write: `plan.md`, `todo.md`, `journal.md`, `assembly_definition.yaml` (planner-owned draft), `benchmark_definition.yaml` (benchmark-owned task fields plus planner-owned internal max targets under benchmark caps)
- Electronics Planner:
  - read: `skills/**`, `utils/**`, `benchmark_definition.yaml`, `plan.md` (if materialized from shared templates), `todo.md` (if materialized from shared templates), `journal.md` (if materialized from shared templates)
  - write: `plan.md`, `todo.md`, `journal.md`, `assembly_definition.yaml` (planner-owned electrical handoff), `benchmark_definition.yaml` (benchmark-owned electrical requirements and planner-owned constraints only)
- Engineering Coder:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `reviews/**`, `renders/**`
  - write: `script.py`, additional `*.py` implementation files, `todo.md` (checkbox progress only), `journal.md`, `renders/**` (tool-generated), `plan_refusal.md` (only when refusing plan)
  - note: this role owns unified implementation, including electrical/wiring logic when the approved plan requires electronics
- Engineering Plan Reviewer:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `plan_refusal.md` (if present), `script.py`, implementation files, `renders/**`, `journal.md`
  - write:
    - `reviews/engineering-plan-review-decision-round-*.yaml`
    - `reviews/engineering-plan-review-comments-round-*.yaml`
- Engineering Execution Reviewer:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `plan_refusal.md` (if present), `script.py`, implementation files, `renders/**`, `journal.md`
  - write:
    - `reviews/engineering-execution-review-decision-round-*.yaml`
    - `reviews/engineering-execution-review-comments-round-*.yaml`
- Benchmark Planner:
  - read: `skills/**`, `utils/**`, benchmark prompt/context inputs, `plan.md` (if materialized from shared templates), `todo.md` (if materialized from shared templates), `journal.md` (if materialized from shared templates)
  - write: `plan.md`, `todo.md`, `journal.md`, `benchmark_definition.yaml` (benchmark-owned), `benchmark_assembly_definition.yaml` (benchmark-owned draft, handed to engineering as read-only context)
- Benchmark Plan Reviewer:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `renders/**`, `journal.md`
  - write:
    - `reviews/benchmark-plan-review-decision-round-*.yaml`
    - `reviews/benchmark-plan-review-comments-round-*.yaml`
- Benchmark Coder:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `reviews/**`, `renders/**`
  - write: `script.py`, additional `*.py` implementation files, `todo.md` (checkbox progress only), `journal.md`, `renders/**` (tool-generated), `plan_refusal.md` (only when refusing plan)
- Benchmark Reviewer:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `plan_refusal.md` (if present), `script.py`, implementation files, `renders/**`, `journal.md`
  - write:
    - `reviews/benchmark-execution-review-decision-round-*.yaml`
    - `reviews/benchmark-execution-review-comments-round-*.yaml`
- Electronics Reviewer:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `plan_refusal.md` (if present), `script.py`, implementation files, `renders/**`, `journal.md`
  - write:
    - `reviews/electronics-review-decision-round-*.yaml`
    - `reviews/electronics-review-comments-round-*.yaml`
- COTS Search subagent:
  - read: `parts.db` (read-only), COTS query helpers/CLI, and any constraints embedded in the single caller-provided request string
  - write: structured COTS result payload returned to caller
- Skill creator/learner:
  - read: `skill-creator/SKILL.md`, existing `skills/**`, relevant journals/reviews/traces, git metadata
  - write: `skills/**` (subject to safety limits), `journal.md`, git commit metadata

System-only metadata:

- `.manifests/**` is reserved for deterministic handover/state metadata written by backend utilities.
- Reviewer-stage manifest filenames are explicit and role-scoped:
  - `.manifests/benchmark_plan_review_manifest.json`
  - `.manifests/benchmark_review_manifest.json`
  - `.manifests/engineering_plan_review_manifest.json`
  - `.manifests/engineering_execution_review_manifest.json`
  - `.manifests/electronics_review_manifest.json`
- Reviewer persistence filenames are explicit and role-scoped:
  - `reviews/benchmark-plan-review-decision-round-<n>.yaml`
  - `reviews/benchmark-plan-review-comments-round-<n>.yaml`
  - `reviews/benchmark-execution-review-decision-round-<n>.yaml`
  - `reviews/benchmark-execution-review-comments-round-<n>.yaml`
  - `reviews/engineering-plan-review-decision-round-<n>.yaml`
  - `reviews/engineering-plan-review-comments-round-<n>.yaml`
  - `reviews/engineering-execution-review-decision-round-<n>.yaml`
  - `reviews/engineering-execution-review-comments-round-<n>.yaml`
  - `reviews/electronics-review-decision-round-<n>.yaml`
  - `reviews/electronics-review-comments-round-<n>.yaml`
- All LLM agent roles are denied read/write/edit on `.manifests/**` via `config/agents_config.yaml`.

Locking rule:

- Before planner submission: planner may edit planner-owned files above.
- After planner submission accepted: benchmark-side `benchmark_definition.yaml` and `benchmark_assembly_definition.yaml` become read-only for benchmark Coder/Reviewer; the same `benchmark_assembly_definition.yaml` is also available in engineer intake as read-only context, while engineer-side `assembly_definition.yaml` becomes read-only for engineering Coder/Reviewer. Only replanning can mutate planner-owned files.
- `benchmark_definition.yaml.benchmark_parts` is benchmark-owned fixture metadata. Coder/Reviewer must treat it as immutable task context, not as solution metadata.

Template files are intentional source artifacts, not ad hoc runtime defaults or symlinks. The reusable common starters are versioned in `shared/agent_templates/common/`, copied into workspaces, and then validated as part of the normal workspace bootstrap and eval/integration materialization path. Note that `skills/` are pulled from git repo (as specified in other parts of the doc).

Another important note: files in e.g. Engineering Coder or Reviewer stages aren't created anew - they are reused from the previous agent.

For submission scripts, direct top-level execution is canonical. The agent should be able to run `python script.py` and have the script perform its own `validate`/`simulate` checks before the review handoff through `utils.submission.submit_for_review(...)`.

`build()` is now a compatibility-only helper rather than a mandatory entrypoint. Runtime may continue to support `build()` while migration is in progress, but new submission-script contracts should not depend on `build()` being present.

Skill-repository boundary is explicit:

1. `skills/` is the canonical runtime skill repository mounted into agent workspaces as `/skills`.
2. `.codex/skills/` is a Codex-only overlay for local debugging/editorial workflows and is not part of the runtime agent skill contract.
3. `suggested_skills/` is a writable sidecar output area for proposed/new skills, not the canonical skill mount that normal agents should read from.
4. Runtime agents should not be taught to treat `.codex/skills/`, `suggested_skills/`, or any other agent-specific skill store as interchangeable with `/skills`.

<!-- Note: the filesystem is not in repo root, but in docker containers. -->

<!-- Note: each of these should be asserted.-->

### Template auto-validation

Where possible, templates have a validation schema. YAML templates in particular should be schema-validated when they are materialized into a workspace, so starter drift is caught at source rather than after a node has already started.

## `agents_config.yaml` (path permissions policy)

To prevent permission drift between "initial files" docs and runtime behavior, we define a centralized policy file at `config/agents_config.yaml`.

The policy uses gitignore-style glob patterns (`*`, `**`, path prefixes) for filesystem access control.

Rules:

01. Every read/write/edit/upload/download operation is checked against this policy in `FilesystemMiddleware`.
02. `deny` takes precedence over `allow`.
03. If a path is not matched by `allow`, access is denied by default.
04. Agent-specific rules override `defaults` (defaults are fallback only).
05. Tool availability can be broad, but path permissions are enforced per role by this file.
06. Reviewer roles (Benchmark Plan Reviewer, Benchmark Reviewer, Engineering Plan Reviewer, Engineering Execution Reviewer, Electronics Reviewer) get `write/edit` tools, but policy only allows writes to their stage-specific persisted review YAML pairs.
07. `.manifests/**` is non-overridable deny for all LLM agent roles (read/write/edit); only backend runtime utilities may access it.
08. No agent role may be granted access to filesystem paths outside its sandbox/workspace root; mounted read-only inputs remain readable only within the sandbox boundary and are not a general filesystem escape hatch.
09. Engineering Plan Reviewer must have tooling to run deterministic handoff checks (`validate_and_price.py` and rule-based DOF scan); if the runtime does not expose this as direct shell execution, it must expose an equivalent dedicated tool with the same fail-closed behavior.
10. Canonical config shape is `filesystem_permissions: {read, write}` under `defaults` and each agent role. Legacy top-level `read`/`write` keys are normalized by runtime loader for backward compatibility, but new edits should use `filesystem_permissions`.
11. The preferred workspace path contract is workspace-relative paths such as `plan.md`, `script.py`, `todo.md`, and `journal.md`. Agent prompts and new code must treat these relative paths as canonical.
12. `/workspace` path aliasing is a compatibility fallback, not a normative contract. It exists because some earlier runtime/tooling behavior accidentally taught agents to address the session root as `/workspace`, and we are temporarily preserving that alias to avoid immediate refactoring cost.
13. We should not expand `/workspace` usage in prompts, tests, or new code. New edits should continue to target the canonical relative-path contract, and the alias should be treated as technical debt scheduled for later cleanup when refactoring cost is acceptable.
14. `config/agents_config.yaml` is also the source of truth for per-role visual-inspection policy, not only path permissions.
15. Visual-inspection policy is structured as `visual_inspection: {required, min_images, reminder_interval}` under each role.
16. Current required-visual roles are: `benchmark_plan_reviewer`, `benchmark_reviewer`, `engineer_planner`, `engineer_coder`, `engineer_plan_reviewer`, and `engineer_execution_reviewer`.
17. Visual inspection is conditional on actual render-image availability in `renders/`; roles are not required to inspect images that do not exist yet.
18. Reminder behavior is runtime-enforced: if a required role keeps working without inspecting the configured minimum number of render images, the runtime periodically injects deterministic reminder messages using `reminder_interval`.
19. The current production/default policy value is `min_images: 1` for the required roles above. This is a policy choice in config, not a hardcoded architecture constant.
20. Long operational guidance should be carried by runtime-loaded skills where possible. `config/prompts.yaml` should define the core contract, but not become the primary home for sprawling workflow instructions.
21. `config/agents_config.yaml` also owns preview-render modality policy under top-level `render: {rgb, depth, segmentation}`.
22. Those flags control whether build123d/VTK-backed preview artifacts are persisted into `renders/` for each modality; they do not change worker routing or backend selection policy.
23. `shared/agent_templates/common/` is the shared source of truth for reusable starter files. Workspace bootstrap, seeded dataset materialization, and integration fixtures copy these template files into the workspace instead of duplicating the same boilerplate in row-local seed bundles.
24. `worker_light/agent_files/` mirrors the same starter content for runtime bootstrap and local inspection. It is a mirrored runtime bundle, not an independent source of truth.
25. When the exact same starter file is reused across agents, keep the body in `shared/agent_templates/common/` and copy it into the workspace from there rather than duplicating it in multiple artifact directories.

Canonical minimal example (`config/agents_config.yaml`):

```yaml
render:
  rgb: true
  depth: true
  segmentation: true

defaults:
  filesystem_permissions:
    read:
      allow: ["**"]
      deny: [".manifests/**"]
    write:
      allow: []
      deny: ["**"]

agents:
  Engineering Planner:
    filesystem_permissions:
      read:
        allow: ["skills/**", "utils/**", "benchmark_definition.yaml", "plan.md", "todo.md", "journal.md"]
        deny: [".manifests/**"]
      write:
        allow: ["plan.md", "todo.md", "journal.md", "assembly_definition.yaml", "benchmark_definition.yaml"]
        deny: ["skills/**", "utils/**", "reviews/**", "renders/**", "script.py", "**/*.py", ".manifests/**"]
    visual_inspection:
      required: true
      min_images: 1
      reminder_interval: 2
```

Legacy expanded example (accepted by loader, not preferred for new edits):

```yaml
version: "1.0"

defaults:
  read:
    allow: []
    deny: [".manifests/**"]
  write:
    allow: []
    deny: [".manifests/**"]

agents:
  Engineering Planner:
    read:
      allow: ["skills/**", "utils/**", "benchmark_definition.yaml", "plan.md", "todo.md", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["plan.md", "todo.md", "journal.md", "assembly_definition.yaml", "benchmark_definition.yaml"]
      deny: ["skills/**", "utils/**", "reviews/**", "renders/**", "script.py", "**/*.py"]

  Engineering Coder:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "assembly_definition.yaml", "reviews/**", "renders/**"]
      deny: [".manifests/**"]
    write:
      allow: ["script.py", "**/*.py", "todo.md", "journal.md", "renders/**", "plan_refusal.md"]
      deny: ["benchmark_definition.yaml", "assembly_definition.yaml", "plan.md", "skills/**", "utils/**", "reviews/**"]

  Engineering Plan Reviewer:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "assembly_definition.yaml", "plan_refusal.md", "script.py", "**/*.py", "renders/**", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["reviews/engineering-plan-review-decision-round-*.yaml", "reviews/engineering-plan-review-comments-round-*.yaml"]
      deny: [".manifests/**"]

  Engineering Execution Reviewer:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "assembly_definition.yaml", "plan_refusal.md", "script.py", "**/*.py", "renders/**", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["reviews/engineering-execution-review-decision-round-*.yaml", "reviews/engineering-execution-review-comments-round-*.yaml"]
      deny: [".manifests/**"]

  Benchmark Planner:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["plan.md", "todo.md", "journal.md", "benchmark_definition.yaml", "benchmark_assembly_definition.yaml"]
      deny: ["skills/**", "utils/**", "reviews/**", "renders/**", "script.py", "**/*.py"]

  Benchmark Plan Reviewer:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "benchmark_assembly_definition.yaml", "renders/**", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["reviews/benchmark-plan-review-decision-round-*.yaml", "reviews/benchmark-plan-review-comments-round-*.yaml"]
      deny: [".manifests/**"]

  Benchmark Coder:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "benchmark_assembly_definition.yaml", "reviews/**", "renders/**"]
      deny: [".manifests/**"]
    write:
      allow: ["script.py", "**/*.py", "todo.md", "journal.md", "renders/**", "plan_refusal.md"]
      deny: ["benchmark_definition.yaml", "benchmark_assembly_definition.yaml", "plan.md", "skills/**", "utils/**", "reviews/**"]

  Benchmark Reviewer:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "benchmark_assembly_definition.yaml", "plan_refusal.md", "script.py", "**/*.py", "renders/**", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["reviews/benchmark-execution-review-decision-round-*.yaml", "reviews/benchmark-execution-review-comments-round-*.yaml"]
      deny: [".manifests/**"]

  COTS Search:
    read:
      allow: ["parts.db", "journal.md", "skills/**", "utils/**"]
      deny: [".manifests/**"]
    write:
      allow: ["journal.md"]
      deny: [".manifests/**"]

  Skill Agent:
    read:
      allow: ["skills/**", "journal.md", "reviews/**"]
      deny: [".manifests/**"]
    write:
      allow: ["skills/**", "journal.md"]
      deny: ["benchmark_definition.yaml", "assembly_definition.yaml", "plan.md", "todo.md", "script.py", "reviews/**"]

  Journalling Agent:
    read:
      allow: ["**"]
      deny: [".git/**", ".env", "**/secrets/**"]
    write:
      allow: ["journal.md"]
      deny: ["**"]

  Electronics Planner:
    read:
      allow: ["skills/**", "utils/**", "benchmark_definition.yaml", "plan.md", "todo.md", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["plan.md", "todo.md", "journal.md", "assembly_definition.yaml", "benchmark_definition.yaml"]
      deny: ["skills/**", "utils/**", "reviews/**", "renders/**", "script.py", "**/*.py"]

  Electronics Reviewer:
    read:
      allow: ["skills/**", "utils/**", "plan.md", "todo.md", "benchmark_definition.yaml", "assembly_definition.yaml", "plan_refusal.md", "script.py", "**/*.py", "renders/**", "journal.md"]
      deny: [".manifests/**"]
    write:
      allow: ["reviews/electronics-review-decision-round-*.yaml", "reviews/electronics-review-comments-round-*.yaml"]
      deny: [".manifests/**"]
```

## Immutability validation

We assert that files (especially "control" files like `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, and `assembly_definition.yaml`) are not edited by coder agents, however they are edited by planner agents. We use git-based hash assertions for such files where they must be immutable.

Control-file ownership split:

1. `benchmark_definition.yaml` owns benchmark/task definition and benchmark fixture metadata (`benchmark_parts`).
2. `benchmark_assembly_definition.yaml` owns benchmark-owned fixture structure, motion metadata, and benchmark-side implementation details. Engineering may read it as immutable benchmark context copied from the benchmark session.
3. `assembly_definition.yaml` owns engineer-planned solution structure, costing inputs, and motion metadata.
4. We do not duplicate engineer solution metadata into `benchmark_definition.yaml`.

Essentially, the goal is a clear separation of concerns: planner files are edited by the planner, review files are edited by reviewer nodes, coder edits coder files; skill subagents modify only skill files, etc.

## File updates

The file writes don't persist in the Controller except into the observability database. Their edits are sent directly into the container mounted (ephemeral) storage.

To update the files, or execute other functions: the controller parses the tool call locally; then sends the request over to the worker to execute whatever is related to the tool - read file, write file, or edit file.

The network latency is perfectly acceptable as LLM reasoning latency far outweighs the ping time.

## Utils files

The agent has a set of utils - python scripts (files) that the agent can import from. These are explicitly unwritable by the `FilesystemMiddleware` (which can force certain files to be unwritable by the agents) the filesystem.

## Skills files

Unlike utils files which are static (or the container restart is acceptable), skills files will be updated by a sidecar learner agent rather frequently (possibly after every 10-20 conversations?).

The container will download the pull skills from a skill git repo before every run (justified? else how do we handle the updates? The file size there is minimal.)

Maybe use a dedicated public git repo? I think it's sound. However this is complexity, and probably isn't that sound. But it'll enable human readability, which is a bonus.

*Note: the skills are to be versioned either way.*

*Note on observability: for observability and reproducibility, the skills usage is to be tracked in the database.*

### Managing skills in git repo

Yes, skills are versioned via a public git repo. Because of filesystem edits and more complex logic, I propose to send git commits and pushes directly from workers.

Default logic: git commit & git push. If push fails due to merge conflict, do git merge. If merge fails, have the skill creator LLM handle it and push it. I suggest using `GitPython` or `git2` (note: we stuck with git2 for the moment) as a library since it's more robust than using shell.

*note*: Ideally, we'd do this logic in controller since controller is the place where we manage all secrets, but... it's not that scary, and we trust our compute nodes, for now.

## Automatic validation of files

Many files - TODO lists, plans, would be automatically verified, with descriptive, markdown-like errors submitted to the agent. (e.g. in plan files, all sections must be submitted).

### Validating markdown files

Markdown is validated statically to ensure a structure and exact match of headings, sometimes content - e.g. bullet points, exact headings or others is met.

In TODO lists, we assert that either [x] or [-] is present in all checkboxes, and no checkboxes were deleted. It *is* pretty inflexible, but I guess that's what we'll do for now. This should keep the model on task longer.

### Validating python files

Files are linted and don't pass execution/submission if they have red errors. Refer to "linting" section.

## Automatic validation on node entry

To prevent unusual bugs in agent state management on entry (e.g., missing plan when entering coder), we add an extra layer of security before node entry - in every node we have a function that validates the input. If it fails, the loops back to the previous graph node.
Notably, during integration tests this simply fail fast as this led to infinite loops.

Planner handoffs that include machine-readable costing totals are part of this node-entry contract. The entry validator must compare the persisted totals against the deterministic script-normalized values and fail closed on any cent-level drift, even when the YAML still parses and the file structure is otherwise valid.

## Starting folder structure for various agents

We define the file structure as follows, individual agents adapt to individual needs:

```text
.
├── skills/                     # [Read-Only] Learned skills and documentation
├── utils/                      # [Read-Only] Fixed utilities and shared code
│   └── ...
├── renders/                      # [Read-Only/tool-generated] (Renders of images and  from the environment) 
│   ├── images/                 # Images from 24 angles (8 images clockwise on 3 levels).
│   └── videos/                 # Video of the simulation. (latest simulation video.)
├── .manifests/                 # [System-Only] Deterministic handover metadata (not readable by LLM agents)
├── reviews/                     # [Read-Only] Reviews of the reviewer agent.
├── journal.md                  # [Read-Write] Decisions, reasoning, and execution log
├── todo.md                     # [Read-Write] Execution plan
├── plan.md                     # A plan
└── script.py                   # [Read-Write] Main execution script (copied from shared/agent_templates/common/)

<!-- The agent can create more than one .py file. -->
```

<!-- Important note: some of these are always fetched from the s3 and are only "fake" for the agent. The "fake" is done by FilesystemMiddleware. -->

### Benchmark generator

#### Benchmark Generator - planner

1. Planning skills
2. A markdown plan template detailing learning objective and, in particular, **geometry** containing (auto-validated, refuses submission if doesn't match template as above)
3. Sample benchmark_definition.yaml (validated)

#### Benchmark Generator - Benchmark Coder

1. Build123d and implementation skills (read-only)
2. A template for creating benchmark generator files (read-write) (e.g. `output.py`)
3. A TODO list (read-write, validated automatically via a on-edit hook (watchdog, or maybe an implicit trigger via ))
4. A plan from the planner (read-only)
5. A journal (read-write)

Utils (read-only):

1. Refuse plan (a script that sends a request to the endpoint) (shouldn't happen, realistically.)
2. Utils necessary for functioning (as described in other parts of the document)

### Engineer

1. Skills (read-only)
2. A template for an engineer files solutions (read-write)
3. A TODO list from the planner (read-write)
4. A plan from the planner (read-only)
5. A Journal (read-write)

Utils (read-only):

1. Refuse plan (a script that sends a request to the endpoint)
2. Utils necessary for functioning (as described in other parts of the document)

### Planner - benchmark generator

## Paralel environments for agents

Notably, a few agents run at the same time on the same machine (because 97% of the time, it's just file updates, so load is tiny, etc.)
As such, the mutliple agents are running in paralel.
To prevent name clashes, we create tempdirs with random names to store agents' filesystems, to prevent filesystem clash and simplify management.

## Agent artifacts

The important part of managing agents are their artifacts.

## Prompts and environment

An agents must know what it is capable to work with. Have a "capabilies.md" document that shows

<!-- Issue found: I tried giving the planner a standard prompt - to move a ball of a diameter 500mm sideways. It created a plan which used Neosprene drive belts, conveyors, aluminum frames - none of which we have. -->

<!-- I didn't have a good LLM to write the propmt. Currently relied on "tricking" the model with a "manufacturing-capability-like" document that shows what it can use.-->

## Journal

The Journal is the agent's **Episodic Memory**. It is a structured log (a constrained Markdown) where the agent records a high-level narrative of its execution.

**Purpose:**

1. To help the agent remember what it did: what it tried and what didn't work:
   a. Intent,
   b. Result,
   c. Reflection,
   d. Next step.
2. To help maintain context amongst task solving retries, and with user conversations.
3. (implicitly, a non-functional requirement) to help debug the agent.
4. (implicitly, a non-functional requirement) to help train the agent
5. (implicitly, a non-functional requirement) to help create skills for the agent.

(note: we should not optimize the journal architecture for the non-functional requirements.)

### Journal lookup for learner agents

The skill learner agent should be able to dig and scrutinize into why something has happened without overflowing their context. The entries in Journal should contain the "start and finish" on the token timeline.

The agents will delimit their reasoning with markdown headings, which would allow easier disclosure on top of the "start and finish".

## TODOs

The Planner will build a list of TODOs for the agent to do. Reviewer nodes will verify against the plan and the TODO list.

## Plan refusal artifact (`plan_refusal.md`)

If a Coder refuses a planner handoff, refusal is only valid with a structured `plan_refusal.md` file.

Rules:

1. `plan_refusal.md` is optional and only created when refusing.
2. The YAML frontmatter contains `reasons` as an array and allows multiple reasons.
3. Reasons are agent-specific and validated by role.
4. The markdown body below frontmatter explains the concrete evidence and what was attempted.

Base frontmatter shape:

```yaml
agent_role: Engineering Coder # or Benchmark Coder
reasons: [cost, weight] # one or more reasons
summary: "Why the current plan cannot be implemented as written"
evidence:
  - "validator output or simulation/log evidence"
  - "which constraint is violated and by how much"
attempted_fixes:
  - "what was tried before refusal"
requested_planner_changes:
  - "specific, actionable changes required to proceed"
```

Allowed reasons by agent role:

- Engineering Coder (unified implementation): `cost`, `weight`, `underspecification`, `incorrect_specification`, `build_zone_violation`, `kinematic_infeasibility`, `manufacturability_conflict`, `schema_inconsistency`, `wiring_infeasibility`, `power_budget_violation`, `component_incompatibility`, `cots_unavailable`
- Benchmark Coder: `underspecification`, `incorrect_specification`, `geometry_conflict`, `objective_conflict`, `invalid_randomization`, `unsolvable_benchmark`, `simulation_invalid`

## Reviews by reviewers

Reviews are stored in `/reviews/` as YAML artifact pairs.

Each reviewer round writes:

1. one decision YAML, which is the routing source of truth,
2. one comments YAML, which carries the reviewer checklist and requested fixes.

The decision YAML uses `decision` enums such as `APPROVED`, `REJECTED`, `REJECT_PLAN`, `REJECT_CODE`, `CONFIRM_PLAN_REFUSAL`, `REJECT_PLAN_REFUSAL`.

If the Benchmark Coder or Engineering Coder refuses the plan, either of `CONFIRM_PLAN_REFUSAL` or `REJECT_PLAN_REFUSAL` can be selected, but only in that context.

The comments YAML contains a canonical stage-specific checklist. Checklist values are `pass | fail | not_applicable`.

Execution-review checklist example:

```yaml
checklist:
  latest_revision_verified: pass
  validation_success: pass
  simulation_success: pass
  visual_evidence_checked: pass
  dynamic_evidence_checked: pass
  plan_fidelity: pass
  robustness: fail
  cost_weight_compliance: pass
  manufacturability_compliance: pass
  dof_deviation_justified: fail
```

Plan-review checklist example:

```yaml
checklist:
  artifacts_complete: pass
  cross_artifact_consistency: pass
  feasible_mechanism: fail
  build_zone_fit: pass
  budget_realism: pass
  cots_valid: pass
  dof_minimality: pass
  ambiguity_free: fail
```

## Token compression

As the agent will near its token generation limits, they will compress their old memory by a summarizing agent.
Compaction is token-based (not character-based) and configured in `config/agents_config.yaml` via `llm.context_compaction_threshold_tokens`.
Runtime persists/broadcasts canonical context telemetry in `metadata.additional_info.context_usage` with `{used_tokens, max_tokens, utilization_ratio}`.

## Skills

The agents will be able to create and update their skills to improve the future runs. This is vital for continuous improvement, and will be used alongside prompt improvement. *Skills* are actually the information that the agent doesn't know and needs to learn - for example how to use `build123d`.

- If the agent fails too much, after resolving a solution, they will persist it to the skill.

We use the `SKILL.md` format as by Anthropic. I refer to ".agent/skill-creator/SKILL.md\` for more information about skills.

We know that agents can not use `build123d` very well despite it being one of the central part of their work in this repo. This is part of a mitigation.

## `build123d` skill and access to documentation

The agents will have an access to build123d documentation through the skill (as per the SKILL.md standard, in \`/references/ folder.)

## Different agents have different skills

The "Engineer" agent does not need a "benchmark creation skill". It could be necessary the other way around. Nevertheless, agents access to skills should be configurable by default. If an agent has created a new skill, let it stay just to that agent.

## Explicit Skill Agent

The agents can not be trusted with updating the skill well, and they may be out of context. Quite likely, implement a separate "learner" agent node that runs after success, probably async of the agent.

It will work over smaller chunks of uncompressed info (how exactly? Maybe progressive disclosure? I think implement a progressive disclosure from a journal)

The Skill Agent will read a `skill-creator/` skill as from Anthropic.

### Skill Agent is run async

The Skill Agent is run asynchronous to the execution, modifying the skill folder and pushing it to a git repo. Its filesystem is managed in the same way as other agents, its outputs stored on (the same) Railway bucket under the folder (because it's easier from the deployment perspective).
The containers will likely have an endpoint to update the skills without restarting. However, for agents, skills are read-only.

### Skill Agent has a journal too

Skill Agent has a journal of:

1. Issues faced and what they resolved to (or weren't resolved)
2. Commonly faced problems (happened more than twice) and solutions to them

If the patterns emerge in the journal; or the solution is obvious enough, the learner writes it into the skill.

#### Structure of the journal

1. Observed struggles (tool calls failed over 4 times)
2. Found solutions to struggles (happened once)
   - The skill builder agent may make mistakes, so, observe the patterns at least twice.
3. Skills to add in the end (happened twice).

Importantly! the agent **must** write an ID of observation and link it to a journal entry. This way, they can:

- Link the found "breakthrough" to an exact problem
- Read the actual reasoning and outcomes from journals instead of putting it to memory.

When writing the skill in the end, they will write a skill from the actual reasoning of the agent, and not be confused.

<!-- Perhaps, make a python script that would add them as entries and make the journal read-only except for the modification of the script? e.g. spec-kitty does this with YAML frontmatter. -->

#### Using a sidecar agent for Journalling

It was proven to be more cost-effective to use a "sidecar" agent that runs in parallel to the primary model. We will use an inexpensive model, such as DeepSeek 3.2 or small models.

<!-- Note: it was found that YAML/Markdown are the most effective for storing model outputs structured information output. YAML, later converted to markdown programmatically is likely preferable? -->

#### When to write in a journal?

As above - when the agent has spent >4 tool calls in a single conversation trying to figure out an issue or syntax, or
This also applies to refactors. If the agent has taken an approach, spent 5 tool calls doing a roundtrip and decided against it in the end, note the architectural issue.
In both cases, if the agent found the solution

The agent will go through all notes in the session and read through ones that are relevant. They will then link the found and solved issues.

<!-- Note: maybe add a "hypothesis" - an agent would be perform better if we X? -->

#### Example

1. Observation: Multiple agents have an issue with struggling to group parts together into a `Compound` - they had more than four unsuccessful tool calls trying to do it (struggle).
2. The learner agent records the issue (async)
3. The main model finally finds that `Compound` syntax requires `Compound(children=[Part|Compound, ...])` syntax (with `children` upfront)
4. The learner agent records the found solution and the skill.

## Worker skills are persisted and are separate from the repository-level skills

Skills in the `.agent/skills/` in the repo root are different from the agent skills we are learning in the database! The repo root skills are for the coding agent to write this codebase. The learned skills should be, e.g. in workspace/ folder.

## Skill safety toggle

The skill writing agent can not delete or overwrite more than 15 lines of code per session (adding is unbound). This is to prohibit a skill being overwritten by any agent completely. Using git diff, if there are more than 15 lines detected, the agent is notified. If the session times out (too many tool calls/the agent finishes despite warnings), we revert the commits.

## Skill versioning

Because the application's performance is quite dependant on SKILL.md files which detail how to use `build123d` and other issues, however those are frequently updated by a "learner" agent, the skill git hashes need to be versioned and persisted to the observability database.

<!-- ### `deepagents` framework
<!-- Note: deepagents was removed -->

## Linting

The agents will receive the linting from the tools. The linting is done on the worker nodes for safety and performance. The agent will have `ruff` and/or `pyright` on the device (I don't think pyrefly is necessary here; standard, non-based pyright will suffice).

The linting will happen at any `write` command; can be triggered by `run_command` command using `ruff`/`pyright`..

## Execution process

The agents (both the engineer and benchmark generator) will create a set of files to solve a given problem. If a benchmark requires 10 different objects, they could create multiple files to solve them, or reuse an existing one.

## Feedback to the agent

It was proven that agents work better with markdown (in general) than JSON (probably due to familiarity with petabytes of text); thus, pass all textual, structured (e.g. text and JSON) feedback in the `markdown` format.

## Feedback from the simulation

The simulation returns the data from *video*. There will be a simple text summary prepended to the video, e.g. "the agent failed to hit the objective"

The agent (engineer, reviewer, or another "summarizer") will write the video summary to the Journal.

### Compressing the video

Future work will need to address the issue of the video being too expensive. Instead, a "T\*" agent could be implemented - a small model that picks the most important shortcuts from the video and feeds it to the agent. This will significantly reduce the pollution of the engineer's agent's attention.

## Feedback from cost and manufacturability constraints

The agent will receive feedback from cost and manufacturability constraints (basically, workbenches) in markdown.
