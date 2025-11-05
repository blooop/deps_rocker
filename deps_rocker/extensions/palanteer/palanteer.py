from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Palanteer(SimpleRockerExtension):
    """Install palanteer profiler for Python and C++ development"""

    name = "palanteer"
    depends_on_extension = ("curl", "git_clone", "x11")

    # Build-time dependencies for compiling palanteer
    builder_apt_packages = [
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
        "git",
    ]

    # Runtime dependencies (libraries needed to run palanteer binaries)
    apt_packages = [
        "libgl1-mesa-dev",
        "libglu1-mesa-dev",
        "libx11-dev",
        "libxrandr-dev",
        "libxinerama-dev",
        "libxcursor-dev",
        "libxi-dev",
    ]
