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
    xvfb \
    xauth \
    libngspice0 \
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
ENV UV_EXTRA_INDEX_URL=https://download.pytorch.org/whl/cpu
ENV UV_LINK_MODE=copy

RUN useradd -m agent
RUN chown -R agent:agent /app

USER agent

RUN --mount=type=cache,target=/home/agent/.cache/uv,uid=1000 \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --frozen --no-install-project --no-dev

COPY --chown=agent:agent worker_renderer ./worker_renderer
COPY --chown=agent:agent worker_heavy ./worker_heavy
COPY --chown=agent:agent shared ./shared
COPY --chown=agent:agent config ./config
COPY --chown=agent:agent pyproject.toml uv.lock ./

RUN --mount=type=cache,target=/home/agent/.cache/uv,uid=1000 \
    uv sync --frozen --no-dev

ENV PYTHONPATH=/app
ENV PATH="/app/.venv/bin:$PATH"

CMD ["xvfb-run", "-a", "uvicorn", "worker_renderer.app:app", "--host", "0.0.0.0", "--port", "8003"]
