# Final Project Report

## Abstract

Current Large Language Models (LLMs) excel at symbolic reasoning and software engineering but frequently fail when tasked with mechanical engineering problems; this is in part due to lack of training data and inability to validate problems. We present **Problemologist**, an end-to-end reinforcement learning environment and an agentic framework designed to bridge this gap. The system employs a dual-agent architecture: a **Benchmark Generator** that autonomously synthesizes physics-based datasets with randomized initial conditions, and an **Engineer Agent** that discovers manufacturable, cost-constrained solutions through iterative CAD drafting, simulation, and Design-for-Manufacturability (DFM) verification. By integrating a high-fidelity simulation engine (Genesis/MuJoCo) with a real-world manufacturing (CNC, Injection Molding, 3D Printing) and cost, weight constraints), Problemologist-AI provides a rigorous platform for evaluating and training the next generation of visual-language models in mechanical design. We open-source our framework at https://github.com/MRiabov/Problemologist-AI.

Keywords: Mechanical Engineering AI, Physical AI, Physics-informed AI, Vision-Language Models datasets, Vision Language Models.

<!--TODO: bake in all project goals into abstract.-->

## Introduction

Mechanical engineering is in nature a very predictable. A part that is be manufactured once will behave exactly like others. However, unlike many other engineering and scientific fields, where public benchmarks are readily available in software engineering [17], physical reasoning [18], materials and chemistry [19], and general machine learning benchmark suites [20], evaluation is exponentially more expensive; and datasets are highly proprietary. Unlike text generation, video, or even robotics, online dataset collection is infeasible due to some of the strictest secrecy and intellectual property restrictions imposed by engineering and manufacturing companies.
Furthermore, even if the datasets existed, no open- or known closed-source reinforcement learning environment which allows high-signal (quality) training of models exists. The previous research focused on "narrow" environments to a specific feature, for example: Finite Element Analysis [27] environments, CAD correctness evaluation and completeness benchmarks [21], narrow robotics environments [2], domain-specific optimization frameworks for airfoils [22] and topology optimization [11], standalone visual benchmarks - of which none allows training a LLM to solve a concrete mechanical or electromechanical task. For example, to create a small machine moving an object from one place to another. Additionally, even if such smaller environments do exist, to our knowledge, no major open-source ML models are post-trained on it, primarily due to lower signal quality of such environments, or being too domain specific.
As such, to advance the results obtained by models in this field, we need a framework that allows to:

1. Robustly validate solutions, first for their theoretical soundness - i.e. hard-and-fast rules used for 3D model CAD correctness, then manufacturing evaluation as well as cost, and weight and other constraints as desired by an engineer (other constraints, e.g., not use actuators to solve a given problem) %% actuators herein equal to motors. %%
2. Validate solutions as they will work in the real world - e.g., to move an object from one position to another - we need to simulate that the object will in fact land in the environment using the mechanism as constructed by a LLM.
   We set out to create a solution that does it and train a system that would solve this set of problems. We also aim to create an architecture that would be highly maintainable and extensible, such that other researchers can extend it to other multiphysics environments and labs can train their models on it.

More specifically, to maximize our contribution to science, our goals are as follows:

1. A production-ready, open-source framework for evaluation and post-training of visual-language models to solve dynamic mechanical engineering and electronics problems, allowing for dataset generation.
2. Create a ready-to-use dataset of engineering problems to solve by agents for post-training,
3. Create a ready-to-use dataset of solutions to the problems with CAD models with manufacturable, realistic and robust solutions.
4. To create a ready-to-use dataset of reasoning traces of current SOTA models solving the tasks be trained during pre-training,
5. To create a dataset of traces trying to solve the problems with manufacturable and verified solutions,
6. Agent Skills acquired during execution and optimizations of the of the model,
7. (additionally) A large library of functional build123d code to enable better pre-training for LLMs on the library (as it will likely serve a pivotal role during open-source development of the mechanical engineering models).

## Related Work

The closest prior work to Problemologist-AI falls into three overlapping groups: embodied benchmark generation, CAD or geometry synthesis, and digital-twin or execution-trace evaluation.

EmboCoach-Bench [28] evaluates LLM agents on developing embodied robots in simulated robotics environments, but its tasks are expert-curated and do not jointly enforce cost, materials, or autonomous task synthesis. EnvScaler [29] and EnvGen [30] show that LLMs can generate or adapt tasks in software and game-like environments, but they remain detached from manufacturable hardware design and from physics-based validation of the resulting mechanism.

A second line of work focuses on CAD or geometry generation. CAD Automation LLMs [31] and STEP-LLM [32] produce FreeCAD scripts or STEP/B-Rep geometry, but their evaluation is largely limited to syntax or geometric fidelity rather than manufacturability, cost compliance, and functional behavior in simulation. Related reasoning-oriented systems such as Cosmos-Reason1 [33] and EnvTrace [34] broaden the scope of embodied reasoning and digital-twin execution, but they remain narrow in scope and do not provide an end-to-end engineering loop from benchmark synthesis to validated assembly. The closest recent work in this direction is the mechanical-designer agent in [27], which combines CAD generation with closed-loop finite-element analysis, but it still does not model cost, weight, or dynamic simulation.

Problemologist-AI differs from these lines of work by combining adversarial benchmark generation, parametric CAD assembly in `build123d`, and simulation-based validation in Genesis/MuJoCo under explicit cost, material, and multi-physics constraints. This makes the system a closed-loop benchmark-and-solution environment rather than a single-task generator or evaluator.

## Methodology

### A synthetic dataset generation system

Similarly to present systems evaluating coding capabilities in greenfield repositories, we need to define a set of problems to be solved by the engineering system.

#### Types of problems to be solved are solving

As this class of problems is readily solvable by major simulation simulation pipelines such as MuJoCo, we first aim to solve multibody dynamics problems. The objective is to: given a predefined starting position of an object, create such a system that would:

- Move it to a concrete position range (an Axis-Aligned Bounding Box (AABB)),
- While avoiding "forbidden" positions, where the machine nor the moved object may not move into (a realistic example is a sensitive area where nothing may go into)
- While adhering to cost, weight, and optionally, electrical constraints.

This problem definition provides for a set of industry relevant systems, such as assembly machinery, manufacturing lines, and robotics.
Our system is also readily extensible to adding fluid simulation, which broadens the scope.

### A selection of machine learning methods

In this section, we study various machine learning methods appropriate for solving mechanical engineering problems.

There are a number of ways how CAD models can be generated:

1. Large Language Models
2. Using pure diffision methods
3. Using custom-trained models, in particular, actor-critic reinforcement learning methods, or training dedicated transformer models to emit CAD and token parameters.
4. Using a combination of diffusion and VLM methods

During early studies, we attempted using custom-trained models, in particular reinforcement learning methods e.g. DQN, PPO, or more advanced world models such as Dreamerv3, however it became challenging as models had no interpretability, and complexity in steering. Just as a software engineer would like to steer code generation, we believe engineers want steerability of CAD models using natural language prompting. In addition, lack of general world knowledge combined with lack of datasets to train on make infeasible to explain to the model, what is, for example, a motor without extensive training. As such, we abandoned this path.

Purely 3d-model generating methods [hunyan3d citation required] suffer from being unable to be edited by an engineer, however it should be noted that such methods are successfully used in topology optimization [citation required]. Additionally, runing a 3d-generating model to output meshes perfectly aligned with manufacturing constraints requires a signficant computational expense. It is also not yet clear how to make a diffusion model align an entire assembly which could consist from dozens, or even hundreds of parts together, though this clearly is a venue for a future work.

A relevant work [https://arxiv.org/pdf/2505.22914] fine-tunes a small model to produce a quality CAD models from only having a small number of images, in spirit similar to advancements in 3d model generation, where a 3d model is generated from a set of images [hunyan3d citation], however, we are constrained by a model that would produce CAD images accurately and aligned to other models, as such we also can't yet employ it. We do believe it could be possible to fine-tune a 2d-generating model to produce relatively accurate models in the future with having enough data.

We choose Large Vision-Language models because they excel at reasoning and agentic, self-corrective behavior, having a significant general knowledge of physics, being relatively easy to operate and tune using prompt optimization methods, we choose VLMs to operate the system.

We also note that all afformentioned methods can be readily adapted to be evaluated in our system - for example, a 3d mesh-generating model can have its outputs inserted into the assembly as an import.

#### LLM & Agentic Infrastructure

##### Translating decisions into actions

Two important findings allow us to use LLMs for engineering: first, code-CAD has emerged and became readily accessible in via tools like build123d [4] %% CodeCAD has existed since 2000s, however build123d makes it materially better in all ways, and provides more industry-standard frameworks %%. An important finding has also emerged during the last three years is that LLMs are significantly superior in using code actions as a way to interact with the environment [23], more so than ReAct [16]. As such, we choose to generate geometric code and other artifacts (e.g. plans) using agentic coding environments, similar to OpenHands framework [9].

##### Separating agents for different tasks

To separate agent concerns and prevent agents from overflowing in a context window, and make evaluation of individual agents both by humans and other agents easier, in line with previous work on multi-agent systems such as AutoGen [24] and CAMEL [25], we separate agents. While a single "general" agent is common in some coding environments, we decompose work into smaller unit problems to keep each stage reviewable and within context limits.

We separate agents into:

- Benchmark generation agents:
  - Benchmark Planner - creating a plan, budgets for costs and weight, planning for off-the-shelf components such as motors, planning for degrees of freedom of individual components, and planning for a design of a given features in benchmarks
  - Benchmark Plan Reviewer - reviewing plans for meeting the above criteria to be met: for example an agent may not invent a part that doesn't exist in the off-the-shelf database, and a a high standard
  - Benchmark Coder - the agent implementing the benchmark plans. Can execute commands.
  - Benchmark Execution Reviewer - an agent reviewing visuals and logical invalidity of a benchmarks.
- Engineering agents:
  - Mechanical Engineering Planner - An agent that plans the mechanism, budgets for costs and weights to drive a solution with minimal expense and to fit the requirements set by the benchmark planner, decides the off-the-shelf components.
  - Electronics Engineering Planner - an agent that plans for electronics components to be present in the environment. Plans for wiring of schematics and then geometrical wiring.
  - Engineering Plan Reviewer - similarly to Benchmark Plan Reviewer, reviews plans for the benchmark. The plans must be able to solve the problems, must be robust upon review, must not use inexistent off-the-shelf components.
  - Engineering Coder - An agent that implements the system. Generates geometric CAD (build123d) code which is verified for manufacturability, cost and weight, and other invalid CAD (e.g. intersecting parts in assembly); if CAD is valid it is translated into simulation artifacts (MuJoCo and Genesis), which can later be simulated for achieving the goal
  - Engineering Solution Reviewer - An agent that works in case of Engineering Coder being incapable.
- Other/infrastructure agents
  - Skill creator agent
  - COTS search agent

##### Agentic planning, research, and adversarial review

To separate agent concerns and prevent agents from overflowing in a context window, and make evaluation of individual agents both by humans and other agents easier, we separate agents, in line with previous work on multi-agent systems such as AutoGen [24] and CAMEL [25]. We adopt the architecture with agents solving smaller unit problems so that each stage remains reviewable and bounded by context limits.

It is more practical for agents to communicate through YAML and Markdown handoff artifacts, so we use those formats for communication; for example, an Engineer Planner supplies the plans and assembly definitions in Markdown and YAML, respectively. Importantly, we employ strict schema validation to make the agents fail fast on almost every file handoff from one agent to another.

#### Simulation

The critical component for our environment and evaluation system is a simulation system. We must be able to validate the agent solutions quickly - as quickly as we can evaluate software, math or physics for agent correctness.

Recent advancements in computing allowed for cheaper and faster simulation for small-scale. While we acknowledge science-grade simulators like FEniCS [7] and Project Chrono [8], it becomes infeasible compiling low-level code on every rerun of a simulation by an agent. In addition, the frameworks are mostly unknown and complex to deal with for machine learning practitioners, in part because they use C++ as their main codebases.
We choose two simulators that fit the purpose of validation fast and efficiently: MuJoCo - a simulator commonly used in robotics simulation, and Genesis simulator. MuJoCo features basic multiphysics simulation, allowing for strong rigid-body problem validation and a familiar interface, while Genesis is a more modern option featuring, among others deformable material simulation. MuJoCo is in particular attractive due to near-zero compilation times, allowing for cheap evaluation, while Genesis stands out for being feature-rich. MuJoCo takes about six seconds to compile for a run up to 5 parts, in our experiments while Genesis takes approximately twelve.

During simulation, we assert that an object has moved into a position as said by the environment, and avoided the forbidden zones as described before. If motors are present, we assert that the motors have not been overloaded. If fluid and electronics are present, and if fluid happens to touch a sensitive piece of electronics, the simulation finishes. Many other features are omitted.
In short, we define a feature-rich simulation that ensures that the simulation will work in the real world.

##### Computer assisted design

We define `build123d` framework as a framework on enabling 3d CAD construction. While other CodeCAD solutions exist, for example OpenSCAD [5] or CadQuery [6], `build123d` importantly allows interoperability with real-world manufacturing solutions, such as STEP file export (a file used by engineers to send manufacturing data to manufacturing companies); while being modern in syntax, having a robust geometry engine under it and having a significant adoption. Critically, the framework is open-source unlike other closed-source solutions, such as SolidWorks.

#### Electronics

### Reward architecture

To optimize a model in a reinforcement learning environment a reward signal is necessary [1]; the GEPA optimizer that we use to optimize our agents - as described below - also uses scoring to optimize prompts. As such, we create a reward system to be optimized against.

We split reward by their functional definition - fast, syntactic checks for file presence and syntactic validity, then "milestones" - actions requiring complex solution, and judge evaluation rewards. For example, the below table describes a reward system for a Engineering Coder agent.

| Section | Key | Weight | Type | Description | Feedback / Formula |
| -- | -- | -: | -- | -- | -- |
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

#### Observability

We store over 60 different event types into our database - events primarily tied to reward signals, as well as tool calls, persisted files/success and failure events. Such a granular evaluation and observability pipeline allows for debugging and monitoring of post-training of models.

### Optimizing the solution by LLMs

A recent discovery on using GEPA (Genetic-Pareto) proves interesting - when the prompt itself is treated as "model weights", we can effectively "train" a model to solve a task without expensive fine-tuning, or even access to model weights. This is further significantly cheaper and grants us access to using closed-source models, which are state-of-the-art, at a minimal expense.

As such, we don't train a model, we optimize the prompts and skills around it. It should be noted that a set of previous work has [focused on fine-tuning models on CAD data](https://sadilkhan.github.io/text2cad-project/) [cadrille citation], however we avoid it as it is again, more expensive, more complex in setup, and because prompt and skill optimization is again, equally effective [26].

#### Prompt tuning

As with other prompt tuning methods, we define a "gold dataset" of evals across each agent and validate pass rates on each agent.

#### Agent skills

A recent introduction of agent skills allows for tuning skills instead of prompts. This is critical in our application - as LLMs are trained poorly on mechanical engineering tools and libraries due to lack of open-source datasets, skills provide us with everything necessary in creating expert systems for CAD operation:

- How to properly write build123d 3d models (something that models are confused at)
- How to search for off-the-shelf components such as actuators and fasteners, structural beams.
- Evaluating designs of other agents for robustness
- Planning for and creating robust solutions
- Electronics knowledge, especially relevant to the current evaluation system
- Critically, creating other skills and editing current skills for continuous learning of agents.

The main advantage is that Skills allow for offloading knowledge from the main prompt, keeping the core prompt compact and easier to maintain, which reduces context pollution.

We update skills during rollouts, self-reflectively. We use an agent going through a `journal.md` file - a compressed history of all reasoning actions of agents with agents.

### Scalabiity and adaptibility

We engineered our solution with extensibility and training scalability in mind.

### Scalability

First and foremost, we employ a distributed architecture, where the simulators such as MuJoCo and Genesis run on the "heavy" worker, "controller" being the "brain" application and "worker-light" serving as a system. This is important to mention because a naive implementation of the system would block LLMs from running if heavy worker is in place. We may expect our solution to be able in handling 100-1000 requests per a single controller and worker-light nodes; with more nodes being devoted to computing the simulation and rendering.

#### Adaptibility and expensibility

The system is made with extensibility in mind. Researchers and practitioners can readily swap our physics simulation backends, agent tools, and add more multiphysics simulation.

## Evaluating the system

While we didn't have enough time to properly tune the model

## References

[1] R. S. Sutton and A. G. Barto, *Reinforcement Learning: An Introduction*, 2nd ed. Cambridge, MA, USA: MIT Press, 2018.

[2] E. Todorov, T. Erez, and Y. Tassa, “MuJoCo: A physics engine for model-based control,” in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2012, pp. 5026–5033.

[3] Genesis Authors, “Genesis: A Generative and Universal Physics Engine for Robotics and Beyond,” GitHub repository, Dec. 2024. [Online]. Available: https://github.com/Genesis-Embodied-AI/Genesis

[4] Build123d Contributors, “build123d,” GitHub repository. [Online]. Available: https://github.com/gumyr/build123d

[5] OpenSCAD Contributors, “OpenSCAD: The Programmers Solid 3D CAD Modeller,” GitHub repository. [Online]. Available: https://github.com/openscad/openscad

[6] CadQuery Contributors, “CadQuery: A Python parametric CAD scripting framework based on OCCT,” Zenodo, doi:10.5281/zenodo.3955118.

[7] M. S. Alnaes *et al*., “The FEniCS Project Version 1.5,” *Archive of Numerical Software*, vol. 3, 2015. [Online]. Available: https://doi.org/10.11588/ans.2015.100.20553

[8] H. Mazhar *et al*., “CHRONO: A parallel multi-physics library for rigid-body, flexible-body, and fluid dynamics,” *Mechanical Sciences*, vol. 4, pp. 49–64, 2013. [Online]. Available: https://doi.org/10.5194/ms-4-49-2013

[9] OpenHands Contributors, “OpenHands: Open platform for AI software developers,” GitHub repository. [Online]. Available: https://github.com/All-Hands-AI/OpenHands

[10] A. Logg, K.-A. Mardal, and G. Wells, *Automated Solution of Differential Equations by the Finite Element Method*. Berlin, Germany: Springer, 2012.

[11] M. P. Bendsøe and O. Sigmund, *Topology Optimization: Theory, Methods, and Applications*. Berlin, Germany: Springer, 2003.

[12] J. Nocedal and S. Wright, *Numerical Optimization*, 2nd ed. New York, NY, USA: Springer, 2006.

[13] A. Bubeck *et al*., “Sparks of artificial general intelligence: Early experiments with GPT-4,” 2023.

[14] S. Khan *et al*., “Text-to-CAD generation through deep learning: A survey,” 2023. [Online]. Available: https://sadilkhan.github.io/text2cad-project/

[15] J. Krishnamurthy, “Constraint-based geometric modeling: Past, present, and future,” *Computer-Aided Design*, vol. 37, no. 10, pp. 1025–1034, 2005.

[16] S. Yao *et al*., “ReAct: Synergizing reasoning and acting in language models,” in *Proc. Int. Conf. Learning Representations (ICLR)*, 2023.

[17] C. E. Jimenez *et al*., “SWE-bench: Can language models resolve real-world GitHub issues?” *arXiv preprint arXiv:2310.06770*, 2023.

[18] D. M. Bear *et al*., “Physion: Evaluating physical prediction from vision in humans and machines,” *arXiv preprint arXiv:2106.08261*, 2021.

[19] A. Dunn *et al*., “Benchmarking materials property prediction methods: The Matbench test set and Automatminer reference algorithm,” *arXiv preprint arXiv:2005.00707*, 2020.

[20] B. Bischl *et al*., “OpenML benchmarking suites,” *arXiv preprint arXiv:1708.03731*, 2017.

[21] K. Alrashedy *et al*., “Generating CAD code with vision-language models for 3D designs,” *arXiv preprint arXiv:2410.05340*, 2024.

[22] N. Bongers, “openfoam-airfoil-optimization: A simple proof-of-concept to optimize airfoils with OpenFOAM and differential evolution,” GitHub repository. [Online]. Available: https://github.com/NielsBongers/openfoam-airfoil-optimization

[23] X. Wang *et al*., “Executable code actions elicit better LLM agents” *arXiv preprint arXiv:2402.01030*, 2024.

[24] Q. Wu *et al*., “AutoGen: Enabling next-gen LLM applications via multi-agent conversation,” *arXiv preprint arXiv:2308.08155*, 2023.

[25] G. Li *et al*., “CAMEL: Communicative agents for ‘Mind’ exploration of large language model society,” *arXiv preprint arXiv:2303.17760*, 2023.

[26] L. A. Agrawal *et al*., “GEPA: Reflective prompt evolution can outperform reinforcement learning,” *arXiv preprint arXiv:2507.19457*, 2025.

[27] Y. Jadhav and A. B. Farimani, “Large language model agent as a mechanical designer,” *arXiv preprint arXiv:2404.17525*, 2024.

[28] Z. Lei *et al*., “EmboCoach-Bench: Benchmarking AI Agents on Developing Embodied Robots,” *arXiv preprint arXiv:2601.21570*, 2026.

[29] X. Song *et al*., “EnvScaler: Scaling Tool-Interactive Environments for LLM Agent via Programmatic Synthesis,” *arXiv preprint arXiv:2601.05808*, 2026.

[30] A. Zala *et al*., “EnvGen: Generating and Adapting Environments via LLMs for Training Embodied Agents,” *arXiv preprint arXiv:2403.12014*, 2024.

[31] S. Kumar *et al*., “Generative AI for CAD Automation: Leveraging Large Language Models for 3D Modelling,” *arXiv preprint arXiv:2508.00843*, 2025.

[32] X. Shi *et al*., “STEP-LLM: Generating CAD STEP Models from Natural Language with Large Language Models,” *arXiv preprint arXiv:2601.12641*, 2026.

[33] NVIDIA, “Cosmos-Reason1: From Physical Common Sense To Embodied Reasoning,” *arXiv preprint arXiv:2503.15558*, 2025.

[34] N. van der Vleuten *et al*., “EnvTrace: Simulation-Based Semantic Evaluation of LLM Code via Execution Trace Alignment -- Demonstrated at Synchrotron Beamlines,” *arXiv preprint arXiv:2511.09964*, 2025.
