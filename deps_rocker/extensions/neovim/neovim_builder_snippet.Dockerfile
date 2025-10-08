# syntax=docker/dockerfile:1.4
@{from deps_rocker.github_helpers import github_release_download}
@(f"ARG NEOVIM_VERSION={NEOVIM_VERSION}")

@(f"FROM {base_image} AS {builder_stage}")
ARG NEOVIM_VERSION

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl tar && \
    rm -rf /var/lib/apt/lists/*

@(github_release_download(
    repo="neovim/neovim",
    asset_pattern="nvim-linux-{arch}.tar.gz",
    cache_dir="/root/.cache/neovim-downloads",
    output_path=f"{builder_output_dir}/nvim",
    extract_cmd="tar -xzf {archive} -C /tmp && cp -a /tmp/nvim-linux-$ARCH {output_path}",
    version_arg="NEOVIM_VERSION",
    cache_id="nvim-downloads"
))
