from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Palanteer(SimpleRockerExtension):
    """Install palanteer profiler for Python and C++ development"""

    name = "palanteer"
    depends_on_extension = ("curl", "git_clone", "x11")
    apt_packages = [
        "build-essential",
        "cmake",
        "python3-dev",
        "python3-pip",
        "libgl1-mesa-dev",
        "libglu1-mesa-dev",
        "libx11-dev",
        "libxrandr-dev",
        "libxinerama-dev",
        "libxcursor-dev",
        "libxi-dev",
    ]
