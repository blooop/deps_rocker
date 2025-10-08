# syntax=docker/dockerfile:1.4
@{from deps_rocker.github_helpers import github_release_download}

@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl tar && \
    rm -rf /var/lib/apt/lists/*

@(github_release_download(
    repo="jesseduffield/lazygit",
    asset_pattern="lazygit_{version}_Linux_{arch}.tar.gz",
    cache_dir="/tmp/lazygit-cache",
    output_path=f"{builder_output_dir}/lazygit",
    extract_cmd="tar -xzf {archive} lazygit && install -Dm755 lazygit {output_path}",
    cache_id="lazygit-release-cache"
))
