from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Cargo(SimpleRockerExtension):
    """Install Rust cargo (via apt for simplicity)"""

    name = "cargo"
