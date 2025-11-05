# syntax=docker/dockerfile:1.4
ARG FZF_VERSION=@FZF_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

# Use cache mount for git clone and fzf installation
RUN --mount=type=cache,target=/root/.cache/fzf-repo,id=fzf-repo-cache \
    bash -c "set -euxo pipefail && \
    # Clone or update repo in cache
    CACHE_DIR='/root/.cache/fzf-repo' && \
    if [ -d \"\$CACHE_DIR/.git\" ]; then \
        cd \"\$CACHE_DIR\" && \
        git fetch origin master && \
        git reset --hard origin/master; \
    else \
        mkdir -p \"\$CACHE_DIR\" && \
        git clone https://github.com/junegunn/fzf.git \"\$CACHE_DIR\"; \
    fi && \
    # Download binary only if it doesn't exist in cache
    cd \"\$CACHE_DIR\" && \
    if [ ! -f bin/fzf ]; then \
        echo 'Binary not cached, running install...' && \
        ./install --bin; \
    else \
        echo 'Using cached fzf binary'; \
    fi && \
    # Copy completed installation to output directory
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR/fzf\" && \
    cp -a \"\$CACHE_DIR/.\" \"\$OUTPUT_DIR/fzf/\""
