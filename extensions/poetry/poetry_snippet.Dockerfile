
# syntax=docker/dockerfile:1.4

# Use BuildKit cache for poetry install
RUN --mount=type=cache,target=/root/.cache/pypoetry,id=poetry-install-cache \
    curl -sSL https://install.python-poetry.org | python3 - && \
    poetry --version

# Copy Poetry binary from builder stage
COPY --from=builder /root/.local/bin/poetry /usr/local/bin/poetry

# Ensure Poetry is executable and available
RUN chmod +x /usr/local/bin/poetry && poetry --version
