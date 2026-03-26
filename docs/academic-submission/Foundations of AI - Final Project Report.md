# Final Project Report

## Abstract

Current Large Language Models (LLMs) excel at symbolic reasoning and software engineering but frequently fail when tasked with mechanical engineering problems; this is in part due to lack of training data and inability to validate problems. We present **Problemologist**, an end-to-end reinforcement learning environment and an agentic framework designed to bridge this gap. The system employs a dual-agent architecture: a **Benchmark Generator** that autonomously synthesizes physics-based datasets with randomized initial conditions, and an **Engineer Agent** that discovers manufacturable, cost-constrained solutions through iterative CAD drafting, simulation, and Design-for-Manufacturability (DFM) verification. By integrating a high-fidelity simulation engine (Genesis/MuJoCo) with a real-world manufacturing (CNC, Injection Molding, 3D Printing) and cost, weight constraints), Problemologist-AI provides a rigorous platform for evaluating and training the next generation of visual-language models in mechanical design. We open-source our framework at https://github.com/MRiabov/Problemologist-AI

Keywords: Mechanical Engineering AI, Physical AI, Physics-informed AI, Vision-Language Models datasets, Vision Language Models.

## Introduction

Mechanical engineering is in nature a very predictable. A part that will be manufactured once will behave exactly like others. However, unlike many other engineering sciences like Software Engineering [citation about ubiquitocy of SWE datasets], Physics [same for physics], Chemistry or even Machine Learning [same for machine learning], evaluation tends to be exponentially more expensive; and datasets are highly proprietary. And unlike text generation, video, or even robotics, dataset collection is infeasible due to strict secrecy and intellectual property restrictions imposed by engineering companies.
Furthermore, even if the datasets existed, no open- or known closed-source environment allowing high-signal (quality) training of models exist. If environments exist, they are generally narrow to a specific feature, for example Finite Element Analysis<sup>1</sup> for CAD correctness evaluation [citation], narrow robotics environments [MuJoCo simulator citation], domain-specific optimization frameworks [citation of aerofoil optimization] [citation of topology optimization frameworks (TopOpt)] standalone visual benchmarks - of which none allows training a LLM to solve a concrete mechanical or other type of engineering tasks - for example, create a small machine moving an object from one place to another. Additionally, even if such smaller environments do exist, to our knowledge, they are not open-source ML models are post-trained on it.
As such, to advance the results obtained by models in this field, we need a framework that allows to:

1. Robustly validate solutions, first for their theoretical soundness - i.e. hard-and-fast rules used for 3D model CAD correctness, then manufacturing evaluation as well as cost, and weight and other constraints as desired by an engineer (other constraints, e.g., not use actuators<sup>2</sup> to solve a given problem) %% actuators herein equal to motors. %%
2. Validate solutions as they will work in the real world - e.g., to move an object from one position to another - we need to simulate that the object will in fact land in the environment using the mechanism as constructed by a LLM.
   We set out to create a solution that does it and train a system that would solve this set of problems. We also aim to create an architecture that would be highly maintainable and extensible, such that other researchers can extend it to other multiphysics environments and labs can train their models on it.

## Related work

[ related work as in the other doc ]
[ findings ]

## Methodology

### Evaluation system

While our evaluation methods can be relatively readily extended to (theoretically) train methods such as 3d-diffusion, LLMs possess an an ability to reason and correct themselves, a significant general knowledge of physics, being relatively easy to operate and tune using prompt optimization methods, we set out to use LLMs to operate the system.

#### LLM & Agentic Infrastructure

##### Translating decisions into actions

Two important findings allow us to use LLMs for engineering: first, code-CAD has emerged and became readily accessible in via tools like build123d<sup>3</sup> %% Code-CAD has existed since 2000s, however build123d makes it materially better in all ways, and provides more industry-standard frameworks %%. An important finding has also emerged during the last three years is that LLMs are significantly superior in using coding tools as a way to interact with the environment [CodeAct citation]. As such, we choose to generate geometric code and other artifacts using agentic coding environment-like too

##### Separating agents for different tasks

To separate agent concerns and prevent agents from overflowing in a context window, and make evaluation of individual agents both by humans and other agents easier, we separate agents, in line with previous work [ ... ]. While a single "do-everything" agent is more common in coding environments, we have taken precaution not to overestimate their capabilities to solve any given problem. We do believe that this is a line for a future research.

We separate agents into:

- Benchmark generation agents:
  - Benchmark Planner - creating a plan, planning for costs, planning for off-the-shelf components such as motors, planning for degrees of freedom of individual compoennts, and planning for a design of a given features in benchmarks
  - Benchmark Plan Reviewer - reviewing plans for meeting the above criteria to a high standard
  - Benchmark Coder<sup>5</sup>
  - Benchmark Reviewer
- Engineering agents:
  - Engineering planner - (to finish with AI)
  - [ ... ]
- Other/infrastructure agents
  - Skill creator agent
  - COTS search agent

##### Agentic planning, research, and adversarial review

To separate agent concerns and prevent agents from overflowing in a context window, and make evaluation of individual agents both by humans and other agents easier, we separate agents, in line with previous work [ citation of other multi-agent systems ]. While a single "do-everything" agent is more common in modern coding environments, this was not the case initially, mostly because in absense of data, it's better to target agents to solve smaller "unit" problems [ citation needed, unverified claim ]. As such, we have taken precaution not to overestimate their capabilities to solve any given problem. We do believe that making a single agent capable of three is a line for a future research.

It is known that agents are reasoning best when outputting YAML files and Markdown outputs, not JSON or XML files [ citation required ] and as such, we facilitate "communication" between agents in YAML and markdown; for example, an engineering coder To make the agents fail fast in case an output is correct

##### 

### Model training

A recent discovery on using self-reflective Generic Pareto system proves interesting - when the prompt itself is treated as "model weights", we can effectively "train" a model to solve a task without expensive fine-tuning, or even access to model weights. This is further significantly cheaper and grants us access to using closed-source models, which are state-of-the-art.

We further update skills, self-reflectively.
