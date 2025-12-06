from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Palanteer(SimpleRockerExtension):
    """Install palanteer profiler for Python and C++ development"""

    name = "palanteer"
    depends_on_extension = ("curl", "git_clone", "x11")

    builder_pixi_packages = [
        "git",
        "cmake",
        "c-compiler",
        "cxx-compiler",
        "make",
        "python",
        "mesa-libgl-devel-cos7-x86_64",
        "mesa-libglu-devel-cos7-x86_64",
        "libx11-devel-cos7-x86_64",
        "libxrandr-devel-cos7-x86_64",
        "libxinerama-devel-cos7-x86_64",
        "libxcursor-devel-cos7-x86_64",
        "libxi-devel-cos7-x86_64",
    ]

    # Build-time dependencies for compiling palanteer
    builder_apt_packages: list[str] = []

    # Runtime dependencies (shared libraries only, no headers)
    apt_packages = [
        "libgl1",  # OpenGL runtime library
        "libglu1-mesa",  # GLU runtime library
        "libgl1-mesa-dri",  # Mesa DRI drivers (includes GPU drivers and software renderer)
        "libx11-6",  # X11 runtime library
        "libxrandr2",  # Xrandr runtime library
        "libxinerama1",  # Xinerama runtime library
        "libxcursor1",  # Xcursor runtime library
        "libxi6",  # Xi runtime library
    ]
