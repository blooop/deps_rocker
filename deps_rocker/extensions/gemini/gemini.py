import os
import pwd
import logging
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Gemini(SimpleRockerExtension):
    """Install Google Gemini CLI tool and mount host ~/.gemini config"""

    name = "gemini"
    depends_on_extension = ("npm", "user")

    def get_docker_args(self, cliargs) -> str:
        """
        Mount host Gemini configuration into the container so the CLI
        can reuse existing authentication and configuration.

        Strategy:
          - Mount ~/.gemini directory if it exists on host
          - Mount current directory .gemini if it exists
          - Preserve environment variables for authentication
        """
        # Determine container home directory (provided by user extension) or fallback
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        if not container_home:
            logging.warning(
                "Could not determine container home directory. Skipping Gemini config mounts."
            )
            return ""

        mounts: list[str] = []
        envs: list[str] = []

        # Mount user-wide gemini config directory
        host_gemini_config = os.path.expanduser("~/.gemini")
        if os.path.exists(host_gemini_config):
            container_gemini_config = f"{container_home}/.gemini"
            mounts.append(f' -v "{os.path.realpath(host_gemini_config)}:{container_gemini_config}"')

        # Mount project-specific .gemini directory if it exists in current working directory
        cwd_gemini_config = os.path.join(os.getcwd(), ".gemini")
        if os.path.exists(cwd_gemini_config):
            mounts.append(f' -v "{os.path.realpath(cwd_gemini_config)}:/workspaces/.gemini"')

        # Preserve important Gemini environment variables
        env_vars = [
            "GEMINI_API_KEY",
            "GOOGLE_API_KEY",
            "GOOGLE_CLOUD_PROJECT",
            "GOOGLE_GENAI_USE_VERTEXAI",
            "GOOGLE_APPLICATION_CREDENTIALS",
        ]

        for env_var in env_vars:
            if env_var in os.environ:
                envs.append(f' -e "{env_var}={os.environ[env_var]}"')

        # Mount Google Application Credentials file if specified and exists
        if "GOOGLE_APPLICATION_CREDENTIALS" in os.environ:
            creds_path = os.environ["GOOGLE_APPLICATION_CREDENTIALS"]
            if os.path.exists(creds_path):
                # Mount the credentials file to the same path in container
                mounts.append(f' -v "{os.path.realpath(creds_path)}:{creds_path}"')

        return "".join(mounts + envs)
