FROM mcr.microsoft.com/devcontainers/base:jammy

# Copy uv and uvx from the official image
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

ARG PIXI_VERSION=v0.48.2

RUN curl -L -o /usr/local/bin/pixi -fsSL --compressed "https://github.com/prefix-dev/pixi/releases/download/${PIXI_VERSION}/pixi-$(uname -m)-unknown-linux-musl" \
    && chmod +x /usr/local/bin/pixi \
    && pixi info

# Install rocker using uvx
RUN uv tool install rocker

# set some user and workdir settings to work nicely with vscode
USER vscode
WORKDIR /home/vscode

RUN echo 'eval "$(pixi completion -s bash)"' >> /home/vscode/.bashrc
