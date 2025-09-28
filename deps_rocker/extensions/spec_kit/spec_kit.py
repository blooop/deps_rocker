from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class SpecKit(SimpleRockerExtension):
    """Install GitHub's Spec Kit for AI-assisted spec-driven development"""

    name = "spec_kit"
    depends_on_extension = ("uv", "git_clone")
