FROM ghcr.io/astral-sh/uv:python3.12-trixie-slim

# Install system dependencies
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
    xvfb \
    xauth \
    libngspice0 \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Enable bytecode compilation
ENV UV_COMPILE_BYTECODE=1

# Copy from the cache instead of linking since it's a separate volume
ENV UV_LINK_MODE=copy

# Create non-root user
RUN useradd -m agent
RUN chown -R agent:agent /app

# Switch to non-root user
USER agent

# Install dependencies
RUN --mount=type=cache,target=/home/agent/.cache/uv,uid=1000 \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --no-install-project --no-dev

# Copy source code
COPY --chown=agent:agent worker ./worker
COPY --chown=agent:agent shared ./shared
COPY --chown=agent:agent pyproject.toml uv.lock ./

# Install the project
RUN --mount=type=cache,target=/home/agent/.cache/uv,uid=1000 \
    uv sync --no-dev

# Set Python path
ENV PYTHONPATH=/app

# Place executables in the path
ENV PATH="/app/.venv/bin:$PATH"

# Entrypoint
CMD ["xvfb-run", "-a", "uvicorn", "worker.app:app", "--host", "0.0.0.0", "--port", "8001"]
