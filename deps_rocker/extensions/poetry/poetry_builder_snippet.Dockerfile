# syntax=docker/dockerfile:1.4
@(f"ARG POETRY_VERSION={POETRY_VERSION}")

@(f"FROM {base_image} AS {builder_stage}")

# Re-declare ARG after FROM to make it available in this stage
@(f"ARG POETRY_VERSION={POETRY_VERSION}")

# Install Poetry in builder stage with cache for installer script and pip packages
RUN --mount=type=cache,target=/root/.cache/poetry-install-cache,id=poetry-install-cache \
    --mount=type=cache,target=/root/.cache/pip,id=pip-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /root/.cache/poetry-install-cache \"\$OUTPUT_DIR\" && \
    script=/root/.cache/poetry-install-cache/install-poetry.py && \
    if [ ! -f \"\$script\" ]; then \
        curl -sSL https://install.python-poetry.org -o \"\$script\"; \
    fi && \
    python3 \"\$script\" --version \$POETRY_VERSION && \
    cp -a /root/.local \"\$OUTPUT_DIR/.local\""
