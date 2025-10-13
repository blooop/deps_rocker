from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Poetry(SimpleRockerExtension):
    """Install Poetry for Python dependency management using builder pattern."""

    name = "poetry"
    depends_on_extension: tuple[str, ...] = ("curl",)
    builder_apt_packages = ["curl", "ca-certificates"]
