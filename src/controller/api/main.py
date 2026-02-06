import os
from fastapi import FastAPI
from pydantic import BaseModel, Field

from src.controller.clients.worker import WorkerClient
from src.controller.graph.agent import create_agent_graph
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware

WORKER_URL = os.getenv("WORKER_URL", "http://worker:8001")

app = FastAPI(title="Problemologist Controller")


class AgentRunRequest(BaseModel):
    task: str = Field(..., description="The task for the agent to perform.")
    session_id: str = Field(..., description="Session ID for the worker.")


def get_worker_client(session_id: str) -> WorkerClient:
    return WorkerClient(base_url=WORKER_URL, session_id=session_id)


@app.get("/")
async def read_root():
    return {"status": "ok", "service": "controller"}


@app.get("/health")
async def health():
    return {"status": "healthy"}


@app.post("/agent/run")
async def run_agent(request: AgentRunRequest):
    client = get_worker_client(request.session_id)
    fs_middleware = RemoteFilesystemMiddleware(client)
    agent = create_agent_graph(fs_middleware)

    # Run the agent
    # For simplicity, we just return the final state/messages
    result = await agent.ainvoke({"messages": [("user", request.task)]})

    return {"status": "completed", "output": result["messages"][-1].content}
