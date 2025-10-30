import os
import pwd
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CachePerms(SimpleRockerExtension):
    """Ensure .cache directory exists with proper permissions for the container user

    This extension creates ~/.cache in the Dockerfile (as root) with 755 permissions,
    which allows the user to write to it. This is necessary for tools like uv, pip, etc.
    to create cache subdirectories without permission errors.

    This must run BEFORE other extensions that mount subdirectories under .cache
    (like claude), so the parent directory exists with correct permissions before
    the volume mount overrides it.
    """

    name = "cache_perms"
    depends_on_extension = ("user",)

    def get_snippet(self, cliargs) -> str:
        """
        Create .cache directory owned by the user with proper permissions.
        This ensures the directory exists before volume mounts, preventing Docker
        from creating it with restrictive permissions. We can't use chown in the
        root snippet, so we use world-writable permissions.
        """
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir

        return f"""
# Create .cache directory with world-writable permissions
# This must exist before volume mounts so Docker doesn't create it as root
# We use 777 to allow the user to write, since we can't easily chown in root context
RUN mkdir -p "{container_home}/.cache" && chmod 777 "{container_home}/.cache"
"""

    def invoke_after(self, cliargs) -> set:
        """Run after user extension is set up"""
        return {"user"}
