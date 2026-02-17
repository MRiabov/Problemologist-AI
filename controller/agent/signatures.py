import dspy
from pydantic import BaseModel, Field
from typing import List, Optional, Any

class ReviewResult(BaseModel):
    decision: str = Field(
        description="Decision: approved, rejected, confirm_plan_refusal, reject_plan_refusal"
    )
    reason: str = Field(description="Reason for the decision")
    required_fixes: List[str] = Field(
        default_factory=list, description="List of required fixes if rejected"
    )

class PlannerSignature(dspy.Signature):
    """
    You are the Lead Mechanical Engineer (Planner).
    Decompose the engineering problem into a structured technical plan.
    Use tools to create 'plan.md', 'todo.md', and 'assembly_definition.yaml'.
    Follow the mandatory reading and order of actions specified in your instructions.
    """
    task = dspy.InputField(desc="The engineering task to solve")
    skills = dspy.InputField(desc="Available skills and knowledge context")
    steer_context = dspy.InputField(desc="Additional steering context from the user")

    plan = dspy.OutputField(desc="Final plan content produced")
    todo = dspy.OutputField(desc="Final todo list produced")

class CoderSignature(dspy.Signature):
    """
    You are the CAD Engineer.
    Execute the technical plan provided by the Planner.
    Write build123d code to create physical parts and assemblies.
    You must respect the Immutable Environment and Physics-Bound Parts constraints.
    Use tools to execute code and verify your design.
    """
    current_step = dspy.InputField(desc="The current step from TODO to implement")
    plan = dspy.InputField(desc="The overall engineering plan")
    error = dspy.InputField(desc="Previous error message if retrying")

    result = dspy.OutputField(desc="Summary of actions taken and result")

class ReviewerSignature(dspy.Signature):
    """
    You are the Design Reviewer (Critic).
    Evaluate the engineer's work after simulation.
    Check for cost compliance, stability, and adherence to the plan.
    Use tools to read reports and inspect artifacts.
    """
    task = dspy.InputField(desc="The original engineering task")
    journal = dspy.InputField(desc="The execution journal of the engineer")
    sim_report = dspy.InputField(desc="Simulation success/failure report hint")
    mfg_report = dspy.InputField(desc="Manufacturing and cost report hint")

    review_result: ReviewResult = dspy.OutputField(
        desc="Structured review decision and feedback"
    )

class ElectronicsEngineerSignature(dspy.Signature):
    """
    You are the Electrical Engineer.
    Design the electrical circuits and route wires for the mechanical assembly.
    Ensure safe and functional circuits using PySpice and check wire clearance.
    Use tools to write electronics.py and validate it.
    """
    task = dspy.InputField(desc="The engineering task and electronic requirements")
    plan = dspy.InputField(desc="The engineering plan")
    cad_implementation = dspy.InputField(desc="The CAD implementation details")

    electronics_code = dspy.OutputField(desc="Summary of the electrical design")

class SkillsSignature(dspy.Signature):
    """
    Analyze the execution journal and task to identify reusable patterns.
    Suggest new skills using the save_suggested_skill tool.
    """
    journal = dspy.InputField(desc="The execution journal")
    task = dspy.InputField(desc="The original task")

    suggested_skills = dspy.OutputField(desc="Description of identified skills")

# Benchmark Signatures

class BenchmarkPlannerSignature(dspy.Signature):
    """
    Expert designer of spatial and geometric puzzles.
    Create a benchmark that trains agents to solve physics problems.
    Use tools to write benchmark_structure.md and benchmark_engineer_todo.md.
    """
    prompt = dspy.InputField(desc="The user request for the benchmark")
    history = dspy.InputField(desc="Conversation history and steering")

    plan: Any = dspy.OutputField(desc="The randomization strategy for the benchmark")

class BenchmarkCoderSignature(dspy.Signature):
    """
    Expert in build123d, Python, and MuJoCo.
    Implement the benchmark from the Planner's strategy.
    Use tools to write script.py and objectives.yaml.
    """
    prompt = dspy.InputField(desc="The user request for the benchmark")
    plan = dspy.InputField(desc="The randomization strategy")
    objectives = dspy.InputField(desc="Current objectives YAML if any")
    feedback = dspy.InputField(desc="Reviewer feedback")
    logs = dspy.InputField(desc="Validation and simulation logs")

    script = dspy.OutputField(desc="Summary of implementation")

class BenchmarkReviewerSignature(dspy.Signature):
    """
    Benchmark Auditor. Review the proposed benchmark for quality and validity.
    Use tools to inspect renders and code.
    """
    theme = dspy.InputField(desc="Benchmark theme")
    prompt = dspy.InputField(desc="Original prompt")
    renders = dspy.InputField(desc="Visual renders hint")
    script = dspy.InputField(desc="Generated build123d script")

    review_result: ReviewResult = dspy.OutputField(desc="Structured review decision")
