ARG NEOVIM_VERSION=v0.11.4
RUN --mount=type=cache,target=/root/.cache/downloads,id=nvim-downloads \
    NVIM_ARCHIVE="/root/.cache/downloads/nvim-${NEOVIM_VERSION}-linux-x86_64.tar.gz" && \
    if [ ! -f "${NVIM_ARCHIVE}" ]; then \
        curl -fsSL "https://github.com/neovim/neovim/releases/download/${NEOVIM_VERSION}/nvim-linux-x86_64.tar.gz" \
             -o "${NVIM_ARCHIVE}"; \
    fi && \
    tar -xzf "${NVIM_ARCHIVE}" -C /opt && \
    ln -sf /opt/nvim-linux-x86_64/bin/nvim /usr/local/bin/nvim
