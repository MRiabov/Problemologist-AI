# Final Project Report

## Abstract

Current Large Language Models (LLMs) excel at symbolic reasoning and software engineering but frequently fail when tasked with mechanical engineering problems; this is in part due to lack of training data and inability to validate problems. We present **Problemologist**, an end-to-end reinforcement learning environment and an agentic framework designed to bridge this gap. The system employs a dual-agent architecture: a **Benchmark Generator** that autonomously synthesizes physics-based datasets with randomized initial conditions, and an **Engineer Agent** that discovers manufacturable, cost-constrained solutions through iterative CAD drafting, simulation, and Design-for-Manufacturability (DFM) verification. By integrating a high-fidelity simulation engine (Genesis/MuJoCo) with a real-world manufacturing (CNC, Injection Molding, 3D Printing) and cost, weight constraints), Problemologist-AI provides a rigorous platform for evaluating and training the next generation of visual-language models in mechanical design. We open-source our framework at https://github.com/MRiabov/Problemologist-AI.

Keywords: Mechanical Engineering AI, Physical AI, Physics-informed AI, Vision-Language Models datasets, Vision Language Models.

<!--TODO: bake in all project goals into abstract.-->

## Introduction

Mechanical engineering is in nature a very predictable. A part that is be manufactured once will behave exactly like others. However, unlike many other engineering sciences like Software Engineering [citation about ubiquitocy of SWE datasets], Physics [same for physics], Chemistry or even Machine Learning [same for machine learning], evaluation is exponentially more expensive; and datasets are highly proprietary. Unlike text generation, video, or even robotics, online dataset collection is infeasible due to some of the strictest secrecy and intellectual property restrictions imposed by engineering and manufacturing companies.
Furthermore, even if the datasets existed, no open- or known closed-source reinforcement learning environment which allows high-signal (quality) training of models exists. The previous research focused on "narrow" environments to a specific feature, for example: Finite Element Analysis<sup>1</sup> environments, CAD correctness evaluation and completeness environments [citation], narrow robotics environments [MuJoCo simulator citation], domain-specific optimization frameworks [citation of aerofoil optimization] [citation of topology optimization frameworks (TopOpt)] standalone visual benchmarks - of which none allows training a LLM to solve a concrete mechanical or electromechanical task. For example, to create a small machine moving an object from one place to another. Additionally, even if such smaller environments do exist, to our knowledge, no major open-source ML models are post-trained on it, primarily due to lower signal quality of such environments, or being too domain specific.
As such, to advance the results obtained by models in this field, we need a framework that allows to:

1. Robustly validate solutions, first for their theoretical soundness - i.e. hard-and-fast rules used for 3D model CAD correctness, then manufacturing evaluation as well as cost, and weight and other constraints as desired by an engineer (other constraints, e.g., not use actuators<sup>2</sup> to solve a given problem) %% actuators herein equal to motors. %%
2. Validate solutions as they will work in the real world - e.g., to move an object from one position to another - we need to simulate that the object will in fact land in the environment using the mechanism as constructed by a LLM.
   We set out to create a solution that does it and train a system that would solve this set of problems. We also aim to create an architecture that would be highly maintainable and extensible, such that other researchers can extend it to other multiphysics environments and labs can train their models on it.

More specifically, to maximize our contribution to science, our goals are as follows:

1. A production-ready, open-source framework for evaluation and post-training of visual-language models to solve dynamic mechanical engineering and electronics problems, allowing for dataset generation.
2. Create a ready-to-use dataset of engineering problems to solve by agents.
3. Create a ready-to-use dataset of solutions to the problems with CAD models with manufacturable, realistic and robust solutions.
4. To create a ready-to-use dataset of reasoning traces of curent SOTA models solving the tasks be trained during pre-training [^1],
5. To create a dataset of traces trying to solve the problems with manufacturable and verified solutions,
6. Agent Skills acquired during execution and optimizations of the of the model,
7. (additionally) A large library of functional build123d code to enable better pre-training for LLMs on the library (as it will likely serve a pivotal role during open-source development of the mechanical engineering models).

## Related work

[ related work as in the other doc ]
[ findings ]

## Methodology

### A synthetic dataset generation system

Similarly to present systems evaluating coding capabilities in greenfield repositories, we need to define a set of problems to be solved by the engineering system.

### Evaluation system

While our evaluation methods can be relatively readily extended to (theoretically) train methods such as 3d-diffusion, LLMs excel at reasoning and agentic, self-corrective behavior, having a significant general knowledge of physics, being relatively easy to operate and tune using prompt optimization methods, we choose LLMs to operate the system.

#### LLM & Agentic Infrastructure

##### Translating decisions into actions

Two important findings allow us to use LLMs for engineering: first, code-CAD has emerged and became readily accessible in via tools like build123d<sup>3</sup> %% Code-CAD has existed since 2000s, however build123d makes it materially better in all ways, and provides more industry-standard frameworks %%. An important finding has also emerged during the last three years is that LLMs are significantly superior in using coding tools as a way to interact with the environment [CodeAct citation]. As such, we choose to generate geometric code and other artifacts (e.g. plans) using agentic coding environments, similar to OpenHands framework [OpenHands]

##### Separating agents for different tasks

To separate agent concerns and prevent agents from overflowing in a context window, and make evaluation of individual agents both by humans and other agents easier, in line with previous work [multiple citations about multi-agent systems], we separate agents. While a single "do-everything" agent is more common in coding environments, we have taken precaution not to overestimate their capabilities to solve any given problem. We do believe that this is a line for a future research, albeit when models will be more capable in this domain.

We separate agents into:

- Benchmark generation agents:
  - Benchmark Planner - creating a plan, planning for costs, planning for off-the-shelf components such as motors, planning for degrees of freedom of individual components, and planning for a design of a given features in benchmarks
  - Benchmark Plan Reviewer - reviewing plans for meeting the above criteria to be met: for example an agent may not invent a part that doesn't exist in the off-the-shelf database, and a a high standard
  - Benchmark Coder<sup>5</sup> - the agent implementing the benchmark plans. Can execute commands.
  - Benchmark Reviewer - an agent reviewing visuals and
- Engineering agents:
  - Engineering planner - (to finish with AI)
  - [ ... ]
- Other/infrastructure agents
  - Skill creator agent
  - COTS search agent

##### Agentic planning, research, and adversarial review

To separate agent concerns and prevent agents from overflowing in a context window, and make evaluation of individual agents both by humans and other agents easier, we separate agents, in line with previous work [ citation of other multi-agent systems ]. While a single "do-everything" agent is more common in modern coding environments, this was not the case initially, mostly because in absense of data, it's better to target agents to solve smaller "unit" problems [ citation needed, unverified claim ]. As such, we have taken precaution not to overestimate their capabilities to solve any given problem. We do believe that making a single agent capable of three is a line for a future research.

It is known that agents are reasoning best when outputting YAML files and Markdown outputs, not JSON or XML files [ citation required ] and as such, we facilitate "communication" between agents in YAML and markdown; for example, an Engineer Planner supplies the plans and assembly definitions in Markdown and YAML, respectively. Importantly, we employ strict schema validation to make the agents fail fast in case an output is correct, on almost every file handoff from an agent to another.

### Reward architecture

To optimize a model in a reinfocement learning environment a reward signal is necessary [Sutton and Barto]; GEPA optimizer that we use to optimize our agents - as described below - also uses scoring to optimize prompts. As such, we create a reward system to be optimized against.

We split reward by their functional definition - fast, synctatic checks for file presence and synctatic validity, then "milestones" - actions requiring complex solution, and judge evaluation rewards. For example, the below table describes a reward system for a Engineering Coder agent.

| Section | Key | Weight | Type | Description | Feedback / Formula |
| -- | -- | -: | -- | -- | -- |
| `engineer_coder` | overall objective | - | - | Scores the CAD engineer's ability to produce a manufacturable, cost-compliant solution that passes simulation. | - |
| `hard_checks` | `script_compiles` | 0.05 | hard check | `script.py` executes without Python syntax or import errors. | Minimum score: `0.02`. Success: the script compiled and executed without syntax or import errors. Failure: the script failed to compile or encountered a runtime error. |
| `hard_checks` | `cad_geometry_valid` | 0.08 | hard check | All parts export to valid OBJ meshes. | Success: CAD geometry is valid and successfully exported. Failure: geometry is invalid, has self-intersections, or failed to export. |
| `hard_checks` | `manufacturability_valid` | 0.07 | hard check | All parts pass workbench manufacturability checks, such as no CNC undercuts and wall thickness above the injection molding minimum. | Success: the design passed all manufacturability constraints. Failure: the design has manufacturability issues, such as undercuts or thin walls. |
| `hard_checks` | `parts_within_build_zone` | 0.05 | hard check | All custom-built parts are fully within the `build_zone` AABB. | Success: all parts are correctly positioned within the designated build zone. Failure: some parts are outside the build zone. |
| `hard_checks` | `cost_within_cap` | 0.10 | partial hard check | Validated unit cost from `validate_and_price` is less than or equal to `max_unit_cost`. | Penalty formula: `max(0.0, 1.0 - max(0.0, actual_cost / max_unit_cost - 1.0))`. Success: manufacturing cost is within budget. Failure: manufacturing cost exceeds the budget. |
| `hard_checks` | `weight_within_cap` | 0.05 | partial hard check | Validated assembly weight is less than or equal to `max_weight_g`. | Penalty formula: `max(0.0, 1.0 - max(0.0, actual_weight / max_weight_g - 1.0))`. Success: assembly weight is within the maximum limit. Failure: assembly weight exceeds the limit. |
| `milestones` | `simulation_result` | 0.45 | partial milestone | Simulation outcome. Full credit is awarded for success; failed runs receive partial credit proportional to progress toward the goal. | Success formula: `0.45`. Failure formula: `0.45 * (1.0 - min_distance_achieved / initial_distance) * 0.4`. Success: the solution solved the problem in simulation. Failure: re-examine mechanical stability, component placement, or movement range. |
| `judge_evaluation` | `reviewer_accepted` | 0.02 | judge signal | The execution reviewer accepted the implementation. | - |
| `judge_evaluation` | `latest_revision_verified` | 0.005 | checklist item | Latest implementation revision was the one evaluated for submission. | - |
| `judge_evaluation` | `validation_success` | 0.01 | checklist item | Validation success is satisfied at review time. | - |
| `judge_evaluation` | `simulation_success` | 0.015 | checklist item | Simulation success is satisfied at review time. | - |
| `judge_evaluation` | `visual_evidence_checked` | 0.005 | checklist item | Required static evidence has been inspected. | - |
| `judge_evaluation` | `dynamic_evidence_checked` | 0.005 | checklist item | Required dynamic evidence has been inspected. | - |
| `judge_evaluation` | `plan_fidelity` | 0.01 | checklist item | Implementation stays aligned with the approved plan. | - |
| `judge_evaluation` | `robustness` | 0.01 | checklist item | Solution is robust rather than passing a single lucky run. | - |
| `judge_evaluation` | `cost_weight_compliance` | 0.005 | checklist item | Final solution stays within cost and weight targets. | - |
| `judge_evaluation` | `manufacturability_compliance` | 0.01 | checklist item | Final solution remains manufacturable under the chosen process. | - |
| `judge_evaluation` | `dof_deviation_justified` | 0.005 | checklist item | Any DOF deviation from the approved plan is justified and reviewable. | - |
| `judge_evaluation` | `feedback_response_score` | 0.03 | partial score | Fraction of valid failed reviewer checklist items addressed on the next coding revision without introducing new blocker regressions. | Partial score. |
| `judge_evaluation` | `bad_feedback_resistance_score` | 0.02 | partial score | Score for following valid reviewer feedback while resisting incorrect or non-applicable requests in mixed-quality review seeds. | Partial score. |

### Optimizing the solution by LLMs

A recent discovery on using self-reflective Generic Pareto system proves interesting - when the prompt itself is treated as "model weights", we can effectively "train" a model to solve a task without expensive fine-tuning, or even access to model weights. This is further significantly cheaper and grants us access to using closed-source models, which are state-of-the-art, at a minimal expense.

As such, we don't train a model, we prompts and skills around it.

#### Prompt tuning

As with other prompt tuning methods, we define a "gold dataset" of evals across each agent and validate pass rates on each agent.

#### Agent skills

A recent introduction of AgentSkill standard [Antropic, 2025] allows for tuning skills instead of prompts. This is critical in our application - as LLMs are trained poorly on mechanical engineering tools and libraries due to lack of open-source datasets, skills provide us with everything necessary in creating expert systems for CAD operation:

- How to properly write build123d 3d models (something that models are confused at)
- How to search for off-the-shelf components such as actuators and fasteners, structural beams.

The main advantage is that Skills allow for offloading knowledge from the main prompt, instead employing "progressive disclosure" [Anthropic, 2025], which reduces context pollution and thus reducing context pollution.

We update skills during rollouts, self-reflectively. We use an agent going through a `journal.md` file - a compressed history of all reasoning actions of agents with agents.

\[^1\]:
