"""Helper functions for downloading GitHub releases with versioned caching"""


def github_release_download(
    repo: str,
    asset_pattern: str,
    cache_dir: str,
    output_path: str,
    extract_cmd: str | None = None,
    version_arg: str | None = None,
    use_jq: bool = False,
    cache_id: str | None = None,
) -> str:
    """
    Generate Dockerfile RUN command for GitHub release download with versioned caching.

    Args:
        repo: GitHub repository (e.g., "jesseduffield/lazygit")
        asset_pattern: Asset name pattern with {version}, {arch}, {platform} placeholders
                      e.g., "lazygit_{version}_Linux_{arch}.tar.gz"
        cache_dir: Directory for caching downloads (e.g., "/tmp/lazygit-cache")
        output_path: Final destination path for the output (file or directory)
        extract_cmd: Custom extraction command. Use {archive} for cache file,
                    {output_dir} for output directory, {output_path} for final path.
                    If None, assumes the archive is a tarball with a single binary.
        version_arg: If using pinned version, the ARG/ENV var name (e.g., "NEOVIM_VERSION").
                    If None, fetches latest release.
        use_jq: Whether to use jq for JSON parsing (robust) or grep/sed (no deps)
        cache_id: BuildKit cache ID. If None, generated from repo name.

    Returns:
        Dockerfile RUN command string with cache mount and versioned download

    Example:
        >>> github_release_download(
        ...     repo="jesseduffield/lazygit",
        ...     asset_pattern="lazygit_{version}_Linux_{arch}.tar.gz",
        ...     cache_dir="/tmp/lazygit-cache",
        ...     output_path=builder_output_dir + "/lazygit",
        ...     extract_cmd="tar -xzf {archive} lazygit && install -Dm755 lazygit {output_path}"
        ... )
    """
    # Generate cache ID from repo if not provided
    if cache_id is None:
        cache_id = f"{repo.replace('/', '-')}-cache"

    # Version detection logic
    if version_arg:
        # Pinned version from ARG/ENV
        version_detection = f'VERSION="${{{version_arg}}}"'
    else:
        # Query latest release from GitHub API
        if use_jq:
            version_detection = f"""release_json=$(curl -fsSL "https://api.github.com/repos/{repo}/releases/latest") && \\
    VERSION=$(echo "$release_json" | jq -r '.tag_name' | sed 's/^v//')"""
        else:
            version_detection = f"""release_json=$(curl -fsSL "https://api.github.com/repos/{repo}/releases/latest") && \\
    VERSION=$(echo "$release_json" | grep -oP '"tag_name":\\s*"\\Kv?[^"]+' | sed 's/^v//')"""

    # Platform/arch detection
    platform_arch_detect = """PLATFORM=$(uname -s | tr '[:upper:]' '[:lower:]') && \\
    ARCH=$(uname -m)"""

    # Map common architecture names
    arch_mapping = """case "$ARCH" in
        x86_64) ARCH_ALT="amd64" ;;
        aarch64|arm64) ARCH_ALT="arm64"; ARCH="aarch64" ;;
        *) ARCH_ALT="$ARCH" ;;
    esac"""

    # Build asset name from pattern
    asset_name_build = f'''ASSET_NAME="{asset_pattern}" && \\
    ASSET_NAME="${{ASSET_NAME//{{version}}/$VERSION}}" && \\
    ASSET_NAME="${{ASSET_NAME//{{platform}}/$PLATFORM}}" && \\
    ASSET_NAME="${{ASSET_NAME//{{arch}}/$ARCH}}"'''

    # Versioned cache logic
    download_logic = f'''CACHE_FILE="{cache_dir}/${{VERSION}}-${{ASSET_NAME}}" && \\
    if [ ! -f "$CACHE_FILE" ]; then \\
        curl -fsSL "https://github.com/{repo}/releases/download/v${{VERSION}}/${{ASSET_NAME}}" -o "$CACHE_FILE"; \\
    fi'''

    # Extraction logic
    if extract_cmd:
        extraction = (
            extract_cmd.replace("{archive}", "$CACHE_FILE")
            .replace("{output_path}", output_path)
            .replace("{output_dir}", f'$(dirname "{output_path}")')
        )
    else:
        # Default: assume tarball with single binary matching the asset name prefix
        binary_name = asset_pattern.split("_")[0] if "_" in asset_pattern else "binary"
        extraction = f'''tar -xzf "$CACHE_FILE" -C /tmp && \\
    install -Dm755 /tmp/{binary_name} "{output_path}"'''

    # Combine into full RUN command
    script = f'''RUN --mount=type=cache,target={cache_dir},id={cache_id} \\
    bash -c "set -euxo pipefail && \\
    mkdir -p {cache_dir} $(dirname {output_path}) && \\
    {version_detection} && \\
    {platform_arch_detect} && \\
    {arch_mapping} && \\
    {asset_name_build} && \\
    {download_logic} && \\
    {extraction}"'''

    return script
