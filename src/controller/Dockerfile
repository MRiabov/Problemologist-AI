FROM python:3.12-slim

# Install uv
RUN pip install uv

# Set working directory
WORKDIR /app

# Copy dependency files
COPY pyproject.toml uv.lock ./

# Install dependencies
RUN uv sync --frozen --no-dev

# Copy source code
COPY src/controller ./src/controller
COPY src/deepagents ./src/deepagents

# Set Python path
ENV PYTHONPATH=/app

# Entrypoint
CMD ["uv", "run", "uvicorn", "src.controller.api.main:app", "--host", "0.0.0.0", "--port", "8000"]
