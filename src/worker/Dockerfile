FROM python:3.12-slim

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglu1-mesa \
    libxrender1 \
    libxext6 \
    libfontconfig1 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Install uv
RUN pip install uv

# Set working directory
WORKDIR /app

# Create non-root user
RUN useradd -m agent
RUN chown -R agent:agent /app
USER agent

# Copy dependency files
COPY --chown=agent:agent pyproject.toml uv.lock ./

# Install dependencies
RUN uv sync --frozen --no-dev

# Copy source code
COPY --chown=agent:agent src ./src

# Set Python path
ENV PYTHONPATH=/app

# Entrypoint
CMD ["uv", "run", "uvicorn", "src.worker.app:app", "--host", "0.0.0.0", "--port", "8001"]