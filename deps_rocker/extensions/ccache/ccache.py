from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Ccache(SimpleRockerExtension):
    """Install ccache and share cache with host (~/.ccache)"""

    name = "ccache"
    apt_packages = ["ccache"]

    def get_docker_args(self, cliargs):
        from pathlib import Path

        # Use host's ccache directory
        ccache_dir = Path.home() / ".ccache"

        # Create directory if it doesn't exist
        ccache_dir.mkdir(exist_ok=True)

        # Mount host ccache directory to container
        return f" -v {ccache_dir}:/root/.ccache"
