from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Palanteer(SimpleRockerExtension):
    """Install palanteer profiler for Python and C++ development"""

    name = "palanteer"
    depends_on_extension = ("curl", "git_clone", "x11")

    # Build-time dependencies for compiling palanteer
    builder_apt_packages = [
        "build-essential",  # gcc, g++, make
        "cmake",  # build system
        "python3-dev",  # Python headers for building bindings
        "git",  # for cloning repository
        # Library headers needed for compilation
        "libgl1-mesa-dev",
        "libglu1-mesa-dev",
        "libx11-dev",
        "libxrandr-dev",
        "libxinerama-dev",
        "libxcursor-dev",
        "libxi-dev",
    ]

    # Runtime dependencies (shared libraries only, no headers)
    apt_packages = [
        "libgl1",  # OpenGL runtime library
        "libglu1-mesa",  # GLU runtime library
        "libx11-6",  # X11 runtime library
        "libxrandr2",  # Xrandr runtime library
        "libxinerama1",  # Xinerama runtime library
        "libxcursor1",  # Xcursor runtime library
        "libxi6",  # Xi runtime library
    ]
