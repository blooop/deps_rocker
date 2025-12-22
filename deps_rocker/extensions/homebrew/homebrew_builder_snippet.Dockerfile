# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

# Install sudo for running installer as non-root user
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/*

# Create non-root user for Homebrew installation
RUN useradd -m -s /bin/bash linuxbrew && \
    echo 'linuxbrew ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Download and install Homebrew with caching
RUN --mount=type=cache,target=/tmp/homebrew-cache,id=homebrew-install-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/homebrew-cache \"\$OUTPUT_DIR\" && \
    chown -R linuxbrew:linuxbrew /tmp/homebrew-cache && \
    installer=/tmp/homebrew-cache/install.sh && \
    if [ ! -f \"\$installer\" ]; then \
        curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh -o \"\$installer\"; \
        chown linuxbrew:linuxbrew \"\$installer\"; \
    fi && \
    su - linuxbrew -c 'NONINTERACTIVE=1 bash /tmp/homebrew-cache/install.sh' && \
    cp -a /home/linuxbrew/.linuxbrew \"\$OUTPUT_DIR/.linuxbrew\" && \
    { \
      echo 'export HOMEBREW_PREFIX=/home/linuxbrew/.linuxbrew'; \
      echo 'export HOMEBREW_CELLAR=/home/linuxbrew/.linuxbrew/Cellar'; \
      echo 'export HOMEBREW_REPOSITORY=/home/linuxbrew/.linuxbrew/Homebrew'; \
      echo 'export PATH=/home/linuxbrew/.linuxbrew/bin:/home/linuxbrew/.linuxbrew/sbin:\$PATH'; \
      echo 'export MANPATH=/home/linuxbrew/.linuxbrew/share/man:\${MANPATH:-}'; \
      echo 'export INFOPATH=/home/linuxbrew/.linuxbrew/share/info:\${INFOPATH:-}'; \
      echo 'export HOMEBREW_NO_ANALYTICS=1'; \
      echo 'export HOMEBREW_NO_AUTO_UPDATE=1'; \
      echo 'export HOMEBREW_ALLOW_ROOT=1'; \
    } > \"\$OUTPUT_DIR/homebrew-env.sh\""
