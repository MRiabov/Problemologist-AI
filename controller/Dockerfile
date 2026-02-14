FROM ghcr.io/astral-sh/uv:python3.12-trixie-slim

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
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
    libngspice0 \
    libvulkan1 \
    libxcursor1 \
    libxinerama1 \
    libxft2 \
    libxrandr2 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

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
COPY worker ./worker
COPY shared ./shared
COPY config ./config
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
