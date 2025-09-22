from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Codex(SimpleRockerExtension):
    """Install OpenAI Codex CLI for AI-assisted development"""

    name = "codex"
    depends_on_extension = ("npm",)
