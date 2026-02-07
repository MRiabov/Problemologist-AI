# Problemologist-AI Codebase Architecture Review #5

This document identifies **NEW** architectural smells and questionable decisions discovered during the fifth deep-dive review of the codebase.

---

## 🔴 Critical Architectural Issues

### 1. Fragile & Risky Dynamic Simulation Execution

**Problem:** `MujocoBridge.run_simulation` constructs a Python script as a raw string and uses `!r` formatting to inject large MJCF XML and agent scripts.

**Evidence:**

- [mujoco_bridge.py:L133-163](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/compiler/mujoco_bridge.py#L133-163)

**The Smell:**

- **In-Process `exec()`**: The sandbox runner script uses `exec(agent_script, namespace)` to run untrusted code. While sandboxed, this pattern makes debugging extremely difficult as line numbers in errors won't match the original script.
- **String Injection Risks**: Using `!r` (repr) on potentially multi-megabyte XML strings or complex Python scripts is fragile. High risk of character escaping issues or memory spikes during string concatenation.
- **Hardcoded Mounting Points**: The string specifies `/workspace` (Line 141, 161) repeatedly. Any change to the sandbox mounting logic requires a surgical sweep of these strings.

**Planned Resolution:**

- **Agent-suggested:**
  - **File-Based Exchange**: Write the XML and agent script to temporary files in the workspace *before* starting the sandbox, and have the runner load them via `Path.read_text()`.
  - **Pre-compiled Control Loops**: Instead of `exec()`, consider a more structured entry point or a protocol-based interaction with the control script.

- **User review**:
    The logic should be:

    1. sandbox starts,
    2. agent writes CAD files in the new temp dir
    3. agent executes the files, and they are parsed to MJCF.
    4. Ideally, mjcf is validated in the same place.

    It should be like that at least. How it is there I don't know, but is it not?

- **Agent response**: Correct. Currently, we side-load strings into the sandbox. I will refactor this to a "File-Based Exchange" where the agent's work is saved to real files in the workspace, and the sandbox simply runs those files. This align better with your 1-2-3-4 logic and solves the debuggability issues.

---

### 2. Tightly Coupled Persistence & Logic

**Problem:** `CADEnv` is inextricably linked to `DatabaseManager`, making it difficult to use in isolation or unit tests.

**Evidence:**

- [core.py:L36-38](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/core.py#L36-38)
- [core.py:L55](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/core.py#L55)

**The Smell:**

- **Hidden Side Effects**: Instantiating a `CADEnv` immediately creates a database file and tables.
- **Dependency Inversion Violation**: `CADEnv` should receive an abstraction of persistence (or a null-persistence layer) rather than concrete `DatabaseManager`.
- **Testing Overhead**: Tests for environment logic are forced to deal with SQLite files and session management.

**Planned Resolution:**

- **Dependency Injection**: Pass the `DatabaseManager` (or an interface) to the constructor. Support a `NullDatabase` for testing.

- **User review**:
    WTF is this? why isn't DB logging central? why would we create a new DB per env? The point of observability is to be able to restore it later, so why would we create a new DB there at all?

- **Agent response**: I completely agree. The current setup is a major architectural leak. I discovered that `CADEnv` is actually **completely vestigial** right now—`set_active_env` is a no-op in the tools, so the agent isn't even using the environment instance created in `solve.py`.

    I will consolidate `CADEnv`'s remaining responsibilities (episode management, task initialization) into a more modern `SessionManager` or directly into `ToolRuntime` and remove the redundant layer.

---

## 🟠 Moderate Design Issues

### 3. Primitive & Non-Scalable Component Recipes

**Problem:** `BDWarehouseProvider` uses a massive, hardcoded `if/elif` chain to generate "recipes" for COTS components.

**Evidence:**

- [bd_warehouse.py:L220-239](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/cots/providers/bd_warehouse.py#L220-239)

**The Smell:**

- **OCP Violation**: Adding a new component type (e.g., a "gear") requires modifying the provider's core logic.
- **Code Duplication**: The logic for instantiating a `StepperMotor` or `HexNut` is repeated in both the `factory` lambda and the `recipe` string.

**Planned Resolution:**

- **Recipe Metadata**: Move recipe generation into the component classes themselves or use a registration-based system where each part type knows how to serialize itself to code.

**User review**:

- Okay... I think you know what you are doing, but first of all we are in python 3.12 and we should for a fact use `match` for all this stuff. Second, I don't know - we won't have that great many classes yet. We could, but not so much.

### 4. Lazy Architectural Coupling

**Problem:** `MujocoBridge` uses lazy imports inside properties to access global configuration and sandbox classes, hiding deep dependencies.

**Evidence:**

- [mujoco_bridge.py:L36](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/compiler/mujoco_bridge.py#L36): `from controller.agent.utils.config import Config`
- [mujoco_bridge.py:L47](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/compiler/mujoco_bridge.py#L47): `from src.environment.sandbox import PodmanSandbox`

**The Smell:**

- **Circularity Smuggling**: These are often used to bypass circular dependency errors, indicating a need for better module separation.
- **Hard-to-Mock**: Testing `MujocoBridge` without a full `Config` or `PodmanSandbox` is difficult because the property will try to instantiate them automatically.

**Planned Resolution:**

- **Constructor Injection**: All required dependencies should be passed via `__init__`. Defaults can be provided, but at the top-level or via a factory.

---

## 🟡 Minor Issues & Smells

### 5. Inconsistent Simulation Feedback Parsing

- Both `CADEnv.dispatch` and `MujocoBridge.run_simulation` contain logic to parse JSON results from the simulation runner. This results in duplicated logic and potential drift in how rewards/energy are calculated or reported.
**User review** - why isn't it in models like @models.py anyway? I don't get it. any reason for it?
- **Agent response**: No good reason—it's just organic growth. I'll move it to `src/compiler/models.py` as a structured parsing method.

### 6. Fragile Keyword-Based Metadata Fallbacks

- `src/cots/utils.py:L53-61` uses keyword matching (`"motor"`, `"bearing"`) to find descriptions if an exact ID match fails. This is highly prone to false positives if part IDs contain these strings in other contexts.

**User Review** - I suppose I'm OK with it. We only have 30-100 or so models in the database... so if the model selects them out of the available options by keywords - I'm OK. I'll upgrade it when necessary. If there is any bad logic outside of this, fix it.

### 7. Hardcoded Project Root Discovery

- `src/agent/utils/prompts.py:L19` uses `.parent.parent.parent.parent` to find the project root. This breaks if the file is moved or if the project is installed as a package. Use shared path utilities.
**user review** - yes, definitely.

### 8. Manual XML Grafting for Meshes

- `MujocoBridge.inject_design` manually creates `asset` and `worldbody` elements using standard `xml.etree`. This is brittle against variations in MJCF structure (e.g. nested includes).

**User review** - do you have any better ideas? <https://github.com/CommonWealthRobotics/mujoco-xml-schema-mjcf> This is the xml schema for MJCF. It could be used for linting!

- **Agent response**: That's a perfect solution. Instead of blind manual grafting, I'll use this schema to validate the final MJCF before it leaves the bridge. This ensures we catch structure errors early and provides much better feedback than MuJoCo's generic load errors.
<!-- TODO test that mjcf leaves the generators correctly. If this XML schema wasn't in the codebase, it means we couldn't have tested.-->

---

## 📊 Summary Table

| Issue | Severity | Effort | Risk |
|-------|----------|--------|------|
| Fragile `exec()` Sim | 🔴 Critical | Medium | Debuggability, Reliability |
| Coupled Persistence | 🔴 Critical | Medium | Testability, Isolation |
| Hardcoded Recipes | 🟠 Moderate | Medium | Maintainability |
| Lazy Imports | 🟠 Moderate | Low | Architectural Clarity |
| Feedback Parsing Drift | 🟡 Minor | Low | Consistency |
| Keyword Fallbacks | 🟡 Minor | Low | Accuracy |
| Root Discovery | 🟡 Minor | Low | Portability |

---

## Recommendations

1. **Protocol-Based Sim**: Replace raw string injection with file-based input/output for the sandbox runner.
2. **DI for Persistence**: Update `CADEnv` to accept a repository/DB interface instead of instantiating it internally.
3. **Self-Describing Parts**: Introduce a `get_recipe()` method on parts or part-providers to decentralize instantiation logic.
4. **Unified Path Utility**: Centralize project root and workspace discovery in a single utility used across all modules.
