import subprocess
import os
import httpx
import structlog
from typing import Any, Optional
from build123d import Compound

logger = structlog.get_logger(__name__)

# Controller API URL - should probably be an env var
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://controller:8000")

class SimulationResult:
    def __init__(self, success: bool, summary: str, video_url: Optional[str] = None):
        self.success = success
        self.summary = summary
        self.video_url = video_url

    def __str__(self):
        return self.summary

def simulate(component: Compound) -> SimulationResult:
    """
    Triggers a physics simulation for the given component.
    
    Steps:
    1. Snapshot current state via Git.
    2. Trigger Simulation Workflow on Controller.
    3. Return summary.
    """
    logger.info("simulation_trigger", type=type(component))
    
    # 1. Git Commit Snapshot
    try:
        # Check if we are in a git repo
        subprocess.run(["git", "rev-parse", "--is-inside-work-tree"], check=True, capture_output=True)
        
        subprocess.run(["git", "add", "."], check=True)
        # We use --allow-empty in case nothing changed
        subprocess.run(["git", "commit", "-m", "Snapshot before simulation", "--allow-empty"], check=True)
        logger.info("git_snapshot_complete")
    except subprocess.CalledProcessError as e:
        logger.warning("git_snapshot_failed", error=str(e))
        # We continue anyway, maybe it's not a git repo or something else
    
    # 2. Trigger Workflow via Controller API
    try:
        # In a real scenario, we might need to serialize the component 
        # or assume it's already saved to a file that the controller can access.
        # For this WP, we focus on the orchestration logic.
        
        response = httpx.post(
            f"{CONTROLLER_URL}/simulation/run",
            json={"session_id": os.getenv("SESSION_ID", "default")},
            timeout=60.0
        )
        response.raise_for_status()
        data = response.json()
        
        workflow_id = data.get("workflow_id")
        logger.info("simulation_workflow_started", workflow_id=workflow_id)
        
        # 3. Poll for result (Simplified for this WP)
        # In a real implementation, this would be a long poll or webhook.
        # We will return a mock result for now as instructed.
        
        summary = """### Simulation Results

**Status**: Passed ✅
**Collision Check**: No collisions detected.
**Stability**: Component is stable under gravity.

[View Simulation Video](https://s3.railway.app/backups/sim_video_latest.mp4)"""
        
        return SimulationResult(
            success=True,
            summary=summary,
            video_url="https://s3.railway.app/backups/sim_video_latest.mp4"
        )
        
    except Exception as e:
        logger.error("simulation_failed", error=str(e))
        return SimulationResult(
            success=False,
            summary=f"Simulation failed: {str(e)}"
        )