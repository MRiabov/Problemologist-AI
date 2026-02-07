FROM ghcr.io/astral-sh/uv:python3.12-trixie-slim

# Set working directory
WORKDIR /app

# Enable bytecode compilation
ENV UV_COMPILE_BYTECODE=1

# Copy from the cache instead of linking since it's a separate volume
ENV UV_LINK_MODE=copy

# Install dependencies
RUN --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --frozen --no-install-project --no-dev

# Copy source code
COPY controller ./controller
COPY shared ./shared
COPY pyproject.toml uv.lock alembic.ini ./

# Install the project
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-dev

# Set Python path
ENV PYTHONPATH=/app

# Place executables in the path
ENV PATH="/app/.venv/bin:$PATH"

# Entrypoint
CMD ["uvicorn", "controller.api.main:app", "--host", "0.0.0.0", "--port", "8000"]
