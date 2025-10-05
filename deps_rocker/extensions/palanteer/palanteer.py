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

    # For builder and snippet templates
    builder_output_dir = "/opt/deps_rocker/palanteer"
    builder_stage = "palanteer_builder"

    empy_args = {
        "builder_stage": builder_stage,
        "builder_output_dir": builder_output_dir,
    }
    empy_builder_args = empy_args
