# Install uv package manager
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Set up uv cache directory
RUN mkdir -p /root/.cache/uv && chmod 755 /root/.cache/uv