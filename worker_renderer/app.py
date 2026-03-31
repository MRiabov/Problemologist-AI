import os
from contextlib import asynccontextmanager

import structlog
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from shared.logging import configure_logging, log_marker_middleware
from shared.rendering import configure_headless_vtk_egl

configure_logging("worker-renderer")
# Force the renderer process onto the OSMesa path even if the image or host
# environment was built with EGL defaults.
os.environ["PYOPENGL_PLATFORM"] = "osmesa"
os.environ["VTK_DEFAULT_OPENGL_WINDOW"] = "vtkOSOpenGLRenderWindow"
configure_headless_vtk_egl()
logger = structlog.get_logger(__name__)
logger.info(
    "renderer_bootstrap_headless_env",
    pyopengl_platform=os.environ.get("PYOPENGL_PLATFORM"),
    vtk_default_open_gl_window=os.environ.get("VTK_DEFAULT_OPENGL_WINDOW"),
)

from worker_renderer.api.routes import (
    is_renderer_busy,
    renderer_busy_context,
    renderer_router,
)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    yield


app = FastAPI(
    title="Problemologist Worker API (Renderer)",
    description="Dedicated headless renderer for static previews and media",
    version="0.1.0",
    lifespan=lifespan,
)
app.add_middleware(log_marker_middleware())
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def log_unhandled_http_exceptions(request: Request, call_next):
    try:
        return await call_next(request)
    except Exception:
        logger.exception(
            "unhandled_http_exception",
            method=request.method,
            path=request.url.path,
            query=str(request.url.query),
        )
        raise


app.include_router(renderer_router, tags=["worker-renderer"])


@app.get("/health")
async def health_check():
    return {"status": "healthy"}


@app.get("/ready")
async def ready_check():
    if is_renderer_busy():
        return JSONResponse(
            status_code=503,
            content={
                "status": "busy",
                "code": "WORKER_BUSY",
                "active_job": renderer_busy_context(),
            },
        )
    return {"status": "ready"}


@app.get("/")
async def root():
    return {
        "name": "Problemologist Worker (Renderer)",
        "version": "0.1.0",
        "docs": "/docs",
    }
