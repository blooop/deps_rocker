ENV PATH="${PATH}:/root/.local/bin"
# Use BuildKit cache for fetching Spec Kit before installation
RUN --mount=type=cache,target=/tmp/spec-kit-git-cache \
    mkdir -p /tmp/spec-kit-git-cache && \
    if [ ! -d /tmp/spec-kit-git-cache/spec-kit ]; then \
        git clone --depth 1 https://github.com/github/spec-kit.git /tmp/spec-kit-git-cache/spec-kit; \
    else \
        branch="$(git -C /tmp/spec-kit-git-cache/spec-kit rev-parse --abbrev-ref HEAD)" && \
        git -C /tmp/spec-kit-git-cache/spec-kit fetch --depth 1 origin "${branch}" && \
        git -C /tmp/spec-kit-git-cache/spec-kit reset --hard "origin/${branch}"; \
    fi && \
    uv tool install specify-cli --from /tmp/spec-kit-git-cache/spec-kit && \
    echo 'Spec Kit installed via uv.'
