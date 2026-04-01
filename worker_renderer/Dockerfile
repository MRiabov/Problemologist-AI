FROM ghcr.io/astral-sh/uv:python3.12-trixie-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libgl1 \
    libglu1-mesa \
    libxrender1 \
    libxext6 \
    libfontconfig1 \
    libx11-6 \
    libegl1 \
    libosmesa6 \
    libglib2.0-0 \
    libsm6 \
    curl \
    dnsutils \
    libvulkan1 \
    vulkan-tools \
    libxcursor1 \
    libxinerama1 \
    libxft2 \
    libxrandr2 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy
ENV PROBLEMOLOGIST_PHYSICS_GL_BACKEND=egl
ENV PROBLEMOLOGIST_RENDER_GL_BACKEND=osmesa
ENV PYOPENGL_PLATFORM=osmesa
ENV PYVISTA_OFF_SCREEN=true
ENV VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow
ENV PYGLET_HEADLESS=1

RUN useradd -m agent
RUN chown -R agent:agent /app

USER agent

COPY --chown=agent:agent worker_renderer/pyproject.toml worker_renderer/uv.lock ./worker_renderer/

RUN python -m venv /app/.venv

ENV VIRTUAL_ENV=/app/.venv
ENV PATH="/app/.venv/bin:$PATH"

RUN --mount=type=cache,target=/home/agent/.cache/uv,uid=1000 \
    uv sync --project worker_renderer --active --frozen --no-install-project --no-dev

COPY --chown=agent:agent worker_renderer ./worker_renderer
COPY --chown=agent:agent shared ./shared
COPY --chown=agent:agent config ./config

ENV PYTHONPATH=/app

CMD ["/app/.venv/bin/uvicorn", "worker_renderer.app:app", "--host", "0.0.0.0", "--port", "8003"]
