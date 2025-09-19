from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Palanteer(SimpleRockerExtension):
    """Build and install Palanteer from source"""

    name = "palanteer"

    def required(self, cliargs) -> set[str]:
        # Needs git and build tools installed via apt in snippet (no extra deps)
        return set()
