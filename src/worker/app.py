import structlog
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .api.routes import router as api_router

# Configure structured logging
structlog.configure(
    processors=[
        structlog.processors.JSONRenderer(),
    ]
)

logger = structlog.get_logger(__name__)

app = FastAPI(
    title="Problemologist Worker API",
    description="Sandboxed execution and filesystem API for agentic CAD tasks",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, this should be restricted
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(api_router, tags=["worker"])

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}

@app.get("/")
async def root():
    """Root endpoint returning API information."""
    return {
        "name": "Problemologist Worker",
        "version": "0.1.0",
        "docs": "/docs"
    }