FROM ghcr.io/astral-sh/uv:python3.12-trixie-slim

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglu1-mesa \
    libxrender1 \
    libxext6 \
    libfontconfig1 \
    libx11-6 \
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

# Copy root config
COPY --chown=agent:agent pyproject.toml uv.lock ./

# Copy packages
COPY --chown=agent:agent shared ./shared
COPY --chown=agent:agent worker ./worker

# Install dependencies
RUN --mount=type=cache,target=/home/agent/.cache/uv,uid=1000 \
    uv sync --frozen --no-dev --package worker

# Set Python path
ENV PYTHONPATH=/app/worker/src:/app/shared/src

# Place executables in the path
ENV PATH="/app/.venv/bin:$PATH"

# Entrypoint
CMD ["uvicorn", "worker.app:app", "--host", "0.0.0.0", "--port", "8001"]
