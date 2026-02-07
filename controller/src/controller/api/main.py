import os
from fastapi import FastAPI
from pydantic import BaseModel, Field, StrictStr
from temporalio.client import Client

from controller.clients.worker import WorkerClient
from controller.graph.agent import create_agent_graph
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.workflows.simulation import SimulationWorkflow
from controller.api.routes import episodes, skills
from shared.enums import ResponseStatus
from shared.logging import configure_logging, get_logger

# Configure logging
configure_logging("controller")
logger = get_logger(__name__)

TEMPORAL_URL = os.getenv("TEMPORAL_URL", "temporal:7233")
WORKER_URL = os.getenv("WORKER_URL", "http://worker:8001")

app = FastAPI(title="Problemologist Controller")

app.include_router(episodes.router)
app.include_router(skills.router)

# Temporal client
temporal_client_instance: Client = None


@app.on_event("startup")
async def startup_event():
    global temporal_client_instance
    try:
        temporal_client_instance = await Client.connect(TEMPORAL_URL)
    except Exception as e:
        logger.error("failed_to_connect_to_temporal", error=str(e))


class AgentRunRequest(BaseModel):
    task: StrictStr = Field(..., description="The task for the agent to perform.")
    session_id: StrictStr = Field(..., description="Session ID for the worker.")


class SimulationRequest(BaseModel):
    session_id: StrictStr = Field(..., description="Session ID for the worker.")
    compound_json: StrictStr = Field(default="{}", description="Component data.")


def get_worker_client(session_id: str) -> WorkerClient:
    return WorkerClient(base_url=WORKER_URL, session_id=session_id)


@app.get("/")
async def read_root():
    return {"status": ResponseStatus.OK, "service": "controller"}


@app.get("/health")
async def health():
    return {"status": ResponseStatus.HEALTHY}


@app.post("/simulation/run")
async def run_simulation(request: SimulationRequest):
    if not temporal_client_instance:
        return {
            "status": ResponseStatus.ERROR,
            "message": "Temporal client not connected",
        }

    handle = await temporal_client_instance.start_workflow(
        SimulationWorkflow.run,
        request.compound_json,
        id=f"sim-{request.session_id}-{os.urandom(4).hex()}",
        task_queue="simulation-task-queue",
    )
    return {"status": ResponseStatus.ACCEPTED, "workflow_id": handle.id}


@app.post("/agent/run")
async def run_agent(request: AgentRunRequest):
    client = get_worker_client(request.session_id)
    # Pass temporal_client to middleware for durable execution
    fs_middleware = RemoteFilesystemMiddleware(
        client, temporal_client=temporal_client_instance
    )
    agent = create_agent_graph(fs_middleware)

    # Run the agent
    result = await agent.ainvoke({"messages": [("user", request.task)]})

    return {
        "status": ResponseStatus.COMPLETED,
        "output": result["messages"][-1].content,
    }
