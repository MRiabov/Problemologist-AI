FROM ghcr.io/astral-sh/uv:python3.12-trixie-slim

# Set working directory
WORKDIR /app

# Enable bytecode compilation
ENV UV_COMPILE_BYTECODE=1

# Copy from the cache instead of linking since it's a separate volume
ENV UV_LINK_MODE=copy

# Copy root config
COPY pyproject.toml uv.lock ./

# Copy packages
COPY shared ./shared
COPY controller ./controller

# Install dependencies for the workspace, focusing on controller
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-dev --package controller

# Set Python path to include our packages
ENV PYTHONPATH=/app/controller/src:/app/shared/src

# Place executables in the path
ENV PATH="/app/.venv/bin:$PATH"

# Entrypoint
CMD ["uvicorn", "controller.api.main:app", "--host", "0.0.0.0", "--port", "8000"]
