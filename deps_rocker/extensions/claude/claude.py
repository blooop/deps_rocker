import os
import pwd
import logging
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Claude(SimpleRockerExtension):
    """Install Claude Code via npm and mount host ~/.claude into the container"""

    name = "claude"
    # Ensure npm is available for installation, and user exists for mounting into home
    depends_on_extension: tuple[str, ...] = ("npm", "user", "uv")

    def get_docker_args(self, cliargs) -> str:
        """
        Mount host Claude configuration/state into the container so the CLI
        behaves the same as on the host and skips first-time setup.

        Strategy:
          - Prefer host XDG config ("$XDG_CONFIG_HOME/claude") if present
          - Else prefer legacy "$HOME/.claude"
          - Resolve symlinks on host before mounting
          - Mount into the container user's home at the same relative path
          - Export CLAUDE_CONFIG_DIR pointing at the mounted path
          - Also mount cache/share dirs if present (best effort)
        """
        # Determine container home directory
        # When user extension is active, it will override the username/home directory
        # For container home, we use the current user's home path structure
        host_userinfo = pwd.getpwuid(os.getuid())
        container_username = (cliargs or {}).get("user_override_name") or host_userinfo.pw_name
        container_home = f"/home/{container_username}"

        # Use host home for determining config paths
        # Always use the actual host user's home for finding config files
        host_home = host_userinfo.pw_dir

        # Log the mounting strategy for debugging
        if container_username != host_userinfo.pw_name:
            logging.debug(
                f"Claude extension: mounting host user '{host_userinfo.pw_name}' config "
                f"to container user '{container_username}'"
            )

        mounts: list[str] = []
        envs: list[str] = []

        # Select host config dir
        host_xdg = os.environ.get("XDG_CONFIG_HOME")
        candidates: list[tuple[str, str]] = []
        if host_xdg:
            candidates.append(
                (os.path.join(host_xdg, "claude"), f"{container_home}/.config/claude")
            )
        candidates.append((os.path.join(host_home, ".claude"), f"{container_home}/.claude"))
        candidates.append(
            (os.path.join(host_home, ".config/claude"), f"{container_home}/.config/claude")
        )

        host_config = None
        container_config = None
        for host_path, container_path in candidates:
            if os.path.exists(host_path):
                host_config = os.path.realpath(host_path)
                container_config = container_path
                break

        if host_config is None:
            logging.warning(
                "No Claude config directory found on host (XDG + ~/.claude). The CLI may run first-time setup in the container."
            )
        else:
            # Mount the entire config directory with proper options
            mounts.append(f' -v "{host_config}:{container_config}"')
            envs.append(f' -e "CLAUDE_CONFIG_DIR={container_config}"')

            # Validate critical config files exist and log their status
            critical_files = [".credentials.json", ".claude.json", "settings.json"]
            accessible_files = []

            for filename in critical_files:
                host_file = os.path.join(host_config, filename)
                if os.path.exists(host_file):
                    # Check if file is readable
                    try:
                        with open(host_file, "r", encoding="utf-8") as f:
                            # Just check readability, don't read content
                            f.read(1)
                        accessible_files.append(filename)
                        logging.debug(f"Claude config file accessible: {filename}")
                    except (IOError, PermissionError) as e:
                        logging.warning(
                            f"Claude config file {filename} exists but not readable: {e}"
                        )

            if accessible_files:
                logging.info(f"Mounting Claude config with accessible files: {accessible_files}")
            else:
                logging.warning(
                    "No accessible Claude config files found - authentication may be required in container"
                )

            # Ensure container config directory exists and has proper permissions
            # This is handled by Docker volume mounting, but we set proper ownership
            container_config_parent = os.path.dirname(container_config)
            if container_config_parent != container_home:
                # Create parent directory structure if using XDG config
                envs.append(f' -e "CLAUDE_ENSURE_CONFIG_DIR={container_config_parent}"')

        # Encourage consistent XDG resolution paths inside the container
        envs.append(f' -e "XDG_CONFIG_HOME={container_home}/.config"')
        envs.append(f' -e "XDG_CACHE_HOME={container_home}/.cache"')
        envs.append(f' -e "XDG_DATA_HOME={container_home}/.local/share"')
        # Do not override PATH globally here; a wrapper ensures ~/.local/bin for claude process

        # Supplemental mounts
        extra_paths = [
            (os.path.join(host_home, ".cache/claude"), f"{container_home}/.cache/claude"),
            (
                os.path.join(host_home, ".local/share/claude"),
                f"{container_home}/.local/share/claude",
            ),
        ]
        for host_extra, container_extra in extra_paths:
            if os.path.exists(host_extra):
                mounts.append(f' -v "{os.path.realpath(host_extra)}:{container_extra}"')

        if not mounts:
            return ""

        return "".join(mounts + envs)

    def get_environment_subs(self, cliargs):
        """
        Get environment setup commands for Claude config.
        """
        # No special setup needed - Docker volume mounting handles permissions
        return {}
