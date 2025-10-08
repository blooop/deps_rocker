@{from deps_rocker.github_helpers import github_release_download}

@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl jq tar && \
    rm -rf /var/lib/apt/lists/*

@(github_release_download(
    repo="openrr/urdf-viz",
    asset_pattern="urdf-viz-{arch}-unknown-linux-gnu.tar.gz",
    cache_dir="/tmp/urdf-viz-cache",
    output_path=f"{builder_output_dir}/urdf-viz",
    extract_cmd="tar -xzf {archive} -C /tmp && find /tmp -name urdf-viz -type f -exec install -Dm755 {{}} {output_path} \\;",
    use_jq=True,
    cache_id="urdf-viz-cache"
))
