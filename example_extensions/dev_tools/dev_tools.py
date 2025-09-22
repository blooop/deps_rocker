from deps_rocker.simple_rocker_extension import SimpleRockerExtension

class DevTools(SimpleRockerExtension):
    """Install essential development tools: vim, htop, curl, git, and build-essential"""

    name = "dev_tools"
    apt_packages = [
        "vim",
        "htop",
        "curl",
        "git",
        "build-essential",
        "ca-certificates"
    ]