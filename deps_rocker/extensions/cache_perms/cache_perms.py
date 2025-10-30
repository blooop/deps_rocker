import os
import pwd
import logging
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CachePerms(SimpleRockerExtension):
    """Ensure .cache directory exists with proper permissions for the container user"""

    name = "cache_perms"
    depends_on_extension = ("user",)

    def get_snippet(self, cliargs) -> str:
        """
        Add a Dockerfile snippet to create .cache with proper ownership.
        This ensures the user can create and use cache directories (e.g., for uv, pip, etc.)
        """
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        username = cliargs.get("user_name") or pwd.getpwuid(os.getuid()).pw_name
        uid = cliargs.get("user_uid") or os.getuid()
        gid = cliargs.get("user_gid") or os.getgid()

        return f"""
# Ensure .cache directory exists with proper ownership for user
RUN mkdir -p "{container_home}/.cache" && \\
    chown {uid}:{gid} "{container_home}/.cache" && \\
    chmod 755 "{container_home}/.cache"
"""

    def invoke_after(self, cliargs) -> set:
        """Run after user extension is set up"""
        return {"user"}
