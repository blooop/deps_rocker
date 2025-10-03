import os
import pwd
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class ClaudeBase(SimpleRockerExtension):
    """Base class for Claude extensions with shared mounting logic"""

    def get_docker_args(self, cliargs) -> str:
        """Mount host ~/.claude directory into container with environment setup"""
        host_userinfo = pwd.getpwuid(os.getuid())
        container_username = (cliargs or {}).get("user_override_name") or host_userinfo.pw_name
        container_home = f"/home/{container_username}"
        host_home = host_userinfo.pw_dir

        # Simple: just mount ~/.claude if it exists
        host_claude = os.path.join(host_home, ".claude")
        if os.path.exists(host_claude):
            return (
                f' -v "{host_claude}:{container_home}/.claude"'
                f' -e "CLAUDE_CONFIG_DIR={container_home}/.claude"'
            )

        return ""
