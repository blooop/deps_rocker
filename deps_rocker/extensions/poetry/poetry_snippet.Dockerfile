# syntax=docker/dockerfile:1.4

@(f"ARG POETRY_VERSION={POETRY_VERSION}")

# Use BuildKit cache mounts for poetry and pip caches
RUN --mount=type=cache,target=/root/.cache/pypoetry \
	--mount=type=cache,target=/root/.cache/pip \
	echo "Poetry and pip caches mounted for faster builds"

# Provide Poetry installation bundle from builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/.local /root/.local")
RUN chmod -R a+rX /root/.local

# Add poetry to PATH and verify installation
ENV PATH="/root/.local/bin:$PATH"
RUN poetry --version
