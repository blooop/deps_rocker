
# syntax=docker/dockerfile:1.4

ARG PYTHON_VERSION=3.11
FROM python:${PYTHON_VERSION}-slim AS builder

# Install dependencies for Poetry installation
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Use BuildKit cache for poetry install
RUN --mount=type=cache,target=/root/.cache/pypoetry,id=poetry-install-cache \
    curl -sSL https://install.python-poetry.org | python3 - && \
    poetry --version

FROM python:${PYTHON_VERSION}-slim AS final

# Copy Poetry binary from builder stage
COPY --from=builder /root/.local/bin/poetry /usr/local/bin/poetry

# Ensure Poetry is executable and available
RUN chmod +x /usr/local/bin/poetry && poetry --version
