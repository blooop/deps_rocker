# syntax=docker/dockerfile:1.4

# Use BuildKit cache for uv cache directory, with a unique id for sharing
RUN --mount=type=cache,target=/root/.cache/uv,id=uv-cache \
	echo "BuildKit cache for uv enabled at /root/.cache/uv"

# Copy uv,uvx binaries from official image
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
