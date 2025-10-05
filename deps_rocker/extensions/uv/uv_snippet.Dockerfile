


# syntax=docker/dockerfile:1.4
# FROM {base_image} AS {builder_stage}  # Provided by main Dockerfile or template engine

# Use BuildKit cache for uv cache directory, with a unique id for sharing
RUN --mount=type=cache,target=/root/.cache/uv,id=uv-cache \
	echo "BuildKit cache for uv enabled at /root/.cache/uv"

# Copy uv binaries from official image
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Optionally, test uv is available (not strictly needed for build, but for debug)
RUN uv --version
