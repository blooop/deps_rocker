import os
import pwd
import logging
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Claude(SimpleRockerExtension):
    """Install Claude Code via install script and mount host ~/.claude into the container"""

    name = "claude"
    # Ensure curl is available for the install script, and user exists for mounting into home
    depends_on_extension: tuple[str, ...] = ("curl", "user")

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
        # Determine container home directory (provided by user extension) or fallback
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        if not container_home:
            logging.warning(
                "Could not determine container home directory. Skipping Claude config mounts."
            )
            return ""

        mounts: list[str] = []
        envs: list[str] = []

        # Select host config dir
        host_xdg = os.environ.get("XDG_CONFIG_HOME")
        candidates: list[tuple[str, str]] = []
        if host_xdg:
            candidates.append(
                (os.path.join(host_xdg, "claude"), f"{container_home}/.config/claude")
            )
        candidates.append((os.path.expanduser("~/.claude"), f"{container_home}/.claude"))
        candidates.append(
            (os.path.expanduser("~/.config/claude"), f"{container_home}/.config/claude")
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
            mounts.append(f' -v "{host_config}:{container_config}"')
            envs.append(f' -e "CLAUDE_CONFIG_DIR={container_config}"')

        # Encourage consistent XDG resolution paths inside the container
        envs.append(f' -e "XDG_CONFIG_HOME={container_home}/.config"')
        envs.append(f' -e "XDG_CACHE_HOME={container_home}/.cache"')
        envs.append(f' -e "XDG_DATA_HOME={container_home}/.local/share"')
        # Do not override PATH globally here; a wrapper ensures ~/.local/bin for claude process

        # Supplemental mounts
        extra_paths = [
            (os.path.expanduser("~/.cache/claude"), f"{container_home}/.cache/claude"),
            (os.path.expanduser("~/.local/share/claude"), f"{container_home}/.local/share/claude"),
        ]
        for host_extra, container_extra in extra_paths:
            if os.path.exists(host_extra):
                mounts.append(f' -v "{os.path.realpath(host_extra)}:{container_extra}"')

        if not mounts:
            return ""

        return "".join(mounts + envs)
