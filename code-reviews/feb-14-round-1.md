# Code Review: `desired_architecture.md` — Feb 14, Round 1

**Scope**: Spec-only review (no code inspection). Focus on inconsistencies, omitted functionality, and underspecifications.

---

## 1. Inconsistencies

### 1.1 Duplicate/conflicting numbered list (L192 vs L242)

The Planner workflow uses numbered steps `1, 2, 3` then restarts at `1` (written as an additional `1.` at L242: "Write required planner artifacts"). This is actually step **4** but is labeled **1**. The markdown renderer will restart the counter, making the document confusing.

### 1.2 `confirm_plan_refusal` duplicated in YAML enum (L972)

```yaml
decision: approved # [approved, rejected, confirm_plan_refusal, confirm_plan_refusal]
```

The enum lists `confirm_plan_refusal` twice instead of including `reject_plan_refusal`. This makes the review decision schema ambiguous — is `reject_plan_refusal` actually a valid option?

### 1.3 `TODO` checkbox format: `[-]` vs `[x]` (L408)

> "we assert that either `[x]` or `[-]` is present in all checkboxes"

Everywhere else the doc uses `[x]` and `[ ]` (standard GFM). The `[-]` notation is non-standard and conflicts with the `[/]` notation you use in spec-kitty. Which is canonical? Should clarify.

### 1.4 Skill safety: "15 lines" vs "5 lines" (L611 vs L1127)

- L611: *"can not delete or overwrite more than **15 lines** of code per session"*
- L1127: *"Skills can not be overwritten for more than **5 lines**"*

These are contradictory thresholds for the same protection mechanism.

### 1.5 Script name inconsistency: `validate_costing_and_price.py` vs `validate_and_price.py`

- L238: `validate_costing_and_price.py`
- L848: `validate_and_price.py`
- L1939-1943: `validate_costing_and_price` (as a planner tool)

The script is referenced under at least two names. One canonical name should be established.

### 1.6 `OBJ` vs `GLB` mesh format confusion (L1646 vs L1652)

- L1646: *"convert build123d CAD files into `obj` format (not STL...)"*
- L1652 (comment): *"Genesis supports GLB"*
- Conversation history shows active work migrating to GLB

The spec says OBJ but doesn't conclusively specify the format. The "not STL" parenthetical also doesn't explain why OBJ over GLB, which is smaller and embeds materials.

### 1.7 Podman vs Docker confusion (L1206)

L1206 says *"We decided to run in Podman containers..."* but L1203 says *"Railway supports docker-compose import and we will start with the docker-compose."* Running terminal output shows `docker compose up`. The spec claims Podman but the actual deployment is Docker.

### 1.8 Worker persistence conflict (L1215 vs L2061)

- L1215: *"we don't store it on controller... upload final results to Railway bucket S3"*
- L2061: *"Any non-file persistence done by worker... on a local SQLite database"*
- L2065: *"We assume that worker databases are ephemeral"*

But earlier L1822: *"persisting a local `events.jsonl` file with all events"*

So workers have: ephemeral SQLite, events.jsonl, AND S3 uploads? The interaction between these three persistence layers is unclear. What goes where?

---

## 2. Omitted Functionality / Gaps

### 2.1 No error handling or retry strategy for agent handovers

The handover flow (L650-665) describes the happy path: planner → CAD → reviewer → next stage. But:

- What happens if the **reviewer times out** or crashes?
- What if the handover message is malformed?
- How many rejection loops are allowed before escalation? (CAD refuses plan, reviewer confirms, planner re-plans — is there a circuit-breaker?)

### 2.2 No specification of the "capabilities.md" document (L483)

> "Have a 'capabilities.md' document that shows..."

This sentence is unfinished. What should `capabilities.md` contain? Which agent types get it? How is it validated? This is critical for preventing agents from planning with non-existent components (the exact issue described in the comment at L485).

### 2.3 Token compression details entirely missing (L523-525)

> "they will compress their old memory by a summarizing agent"

No details on:

- What triggers compression (token count threshold? turn count?)
- What is preserved vs discarded
- Whether the summary agent is the same as the journaling sidecar
- How compressed context interacts with skill lookups or journal references

### 2.4 No authentication/authorization model for multi-user (L2173)

> "Secure all endpoints with an API key env var"

This is single-tenant. The spec mentions "per-user memory" (L1016), "user thumbs up/down" (L1844), and a frontend — implying multi-user. But there's no auth flow, no user model, no session management specified.

### 2.5 Benchmark-to-Engineer "indirect contact" undefined (L656-657)

> "passes the environment to the 'lead engineer' — indirect contact — no actual 'communication'"

How does this handover work in practice? Is it:

- A database record that triggers a new episode?
- A message queue?
- The frontend queuing it?
- Temporal orchestrating it?

This is a critical integration point with zero implementation detail.

### 2.6 `preview_design` tool incomplete spec (L1919-1920)

> "Note - used by default by"

Sentence is unfinished. Used by default by whom? Under what conditions?

### 2.7 No spec for the reviewer's image/video viewing workflow

The reviewer is supposed to check "at least 3 images" (L1076) and review video. But:

- How does the reviewer access renders from the S3 bucket?
- What tool does the reviewer use to "view" images? (the tool list gives filesystem tools, not image viewing tools)
- Are images embedded in the conversation context, or referenced by path?

### 2.8 No conflict resolution for concurrent skill writes (L393-396)

The spec says: *"git commit & git push. If push fails due to merge conflict, do git merge."*

But what if two skill agents running on different workers both edit the same SKILL.md? The merge strategy ("have the skill creator LLM handle it") is vague. What model? What prompt? What if it loops?

### 2.9 Simulation result data structure (`SimulationResult`) undefined

`simulate()` returns `SimulationResult` (L1914, L1932) but its schema is never defined. What fields does it contain? At minimum it should include:

- Pass/fail status
- Failure reason (from the enum at L1750-1755)
- Final positions of all bodies
- Simulation time
- Video path (if rendered)
- Per-seed results (since multiple jitter runs are mandated)

### 2.10 No definition of "episode" lifecycle

The word "episode" appears several times (L1173, L1177, L2126) but is never defined. Is an episode:

- One benchmark generation?
- One engineering attempt?
- One planner → CAD → reviewer cycle?
- One full benchmark → engineering pipeline?

---

## 3. Underspecifications

### 3.1 COTS catalog: "SQL catalog built from `bd_warehouse`" but no schema (L318-326)

The spec says what the catalog is *built from* but not:

- Table schema (what columns exist)
- Query capabilities (full-text search? parametric search?)
- How frequently it's rebuilt
- What happens when a part exists in CAD but not in the catalog

### 3.2 Coordinate system origin (L1095-1096 — noted by author)

The spec's own comment says: *"FIXME: Underspec: we don't define coordinate system starting point"*. This affects every geometric reference in objectives.yaml.

### 3.3 FEM / stress validation scoping (L1641)

> "break upon stress which is higher than max stress - safety factor (note: not applicable for now as we are simulating rigid-body only)"

But the conversations show active work on FEM validation tests. Is FEM in scope or not? The spec says "not applicable for now" but doesn't define when/how it would be enabled.

### 3.4 `drillable` flag propagation (L1422-1429)

The spec says benchmark agents produce `drillable=True/False` metadata but doesn't specify:

- Where this is stored (in the MJCF? in objectives.yaml? in a separate config?)
- How the engineering planner receives it (YAML file? visual overlay?) — L1424 mentions "a YAML file" and "texture or separate set of renders" but doesn't commit to either
- How drilling validation works programmatically

### 3.5 Multiple simulations for runtime jitter — how many? (L1628)

> "run the simulation multiple times (e.g. 5)"

Is 5 the actual number or a placeholder? Is it configurable? Is it the same for benchmark validation and engineer validation?

### 3.6 Video lifecycle and deletion policy (L1237)

> "videos will be automatically deleted after a short period, e.g. a day"

But the spec also says videos are evidence for reviewers and observability. If a review happens asynchronously or a debugging session occurs after 24h, the evidence is gone. No archival policy is defined.

### 3.7 How does the agent know build zone boundaries at construction time? (L1583)

The agent must not construct outside the build zone. But:

- Is this enforced at CAD time (static analysis of coordinates)?
- Or only at simulation/validation time?
- What happens if a part is *partially* outside the build zone (e.g., a fillet extends 0.1mm beyond)?

### 3.8 `manufacturing_config.yaml` — materials section is skeletal (L1701-1708)

```yaml
materials:
  alu-6061:
    color: #
    elongation_stress:
    restitution: 
    friction_coef: 
    # and others
```

All values are blank. The spec doesn't define:

- Full list of required material properties
- Which properties are used by simulation vs. by pricing
- Units for each property

### 3.9 Temporal workflow boundaries undefined (L1257-1263)

> "Temporal is used to orchestrate the workers"

But which operations are Temporal activities? The spec mentions "operations expected to run for more than 30 seconds" (L2122) as a heuristic, but never enumerates the actual workflow definitions. This makes it impossible to know what's durable and what isn't.

### 3.10 Frontend spec punt (L2147-2149)

> "Frontend specs now live in the sibling `frontend-specs.md` file"

The architecture doc mentions user interactions extensively (steering, thumbs up/down, @-mentions, code selection, face/edge selection in 3D viewer, assembly tree), but delegates all of this to another file. This creates a risk that the frontend spec diverges from what this doc promises.

---

## 4. Structural / Readability Issues

### 4.1 Unfinished HTML comments used as TODO markers

There are ~40+ HTML comments (`<!-- -->`) throughout, many containing open questions, uncertainties, or deferred decisions. Examples:

- L53: `<!-- I will need to experiment -->`
- L57: `<!-- note - not sure if I handle dynamic objects atm -->`
- L327: `<!-- TODO move details away from this section -->`
- L967: `<!-- I need to ideate/remember what else it should review for -->`

These are valuable notes but should be surfaced into a dedicated "Open Questions / Deferred Decisions" section rather than scattered inline.

### 4.2 Empty or stub sections

- L67: **"Subagents output requirements"** — heading with no content
- L1147: **"Engineering Planner"** (under Slow evals) — heading with no content
- L1160-1162: **"Reviewer"** — `"Plans not rejected by"` — sentence fragment, no content
- L1180: **"###"** — empty heading

### 4.3 Mixed voice: AI-generated vs human-written

Several sections are explicitly called out as "AI-generated" or "LLM-suggested" (L1509, L1571, L1794). While honest, this creates ambiguity: are these sections reviewed and approved, or are they aspirational drafts? Should they be treated as binding spec?

### 4.4 Complexity tracking worksheet at the end (L2193-2221)

This trailing YAML block is valuable but uses an unusual format (YAML inside a markdown code fence with multi-line strings). It reads like scratch notes rather than spec. Consider either formalizing it into proper sections or removing it to a separate file.

---

## 5. Summary of Highest-Priority Items

| Priority | Issue | Section |
|----------|-------|---------|
| 🔴 High | `SimulationResult` schema undefined | §2.9 |
| 🔴 High | Benchmark→Engineer handover mechanism undefined | §2.5 |
| 🔴 High | `capabilities.md` undefined (agents plan with non-existent parts) | §2.2 |
| 🔴 High | Coordinate system origin undefined (author's own FIXME) | §3.2 |
| 🟡 Medium | Skill safety line threshold contradicts itself (15 vs 5) | §1.4 |
| 🟡 Medium | Script naming inconsistency (`validate_costing_and_price` vs `validate_and_price`) | §1.5 |
| 🟡 Medium | Review YAML enum has duplicate value | §1.2 |
| 🟡 Medium | Token compression completely unspecified | §2.3 |
| 🟡 Medium | Episode lifecycle never defined | §2.10 |
| 🟡 Medium | Reviewer image/video access mechanism unspecified | §2.7 |
| 🟢 Low | Podman vs Docker inconsistency | §1.7 |
| 🟢 Low | Empty headings and sentence fragments | §4.2 |
| 🟢 Low | ~40 inline HTML comment TODOs | §4.1 |
