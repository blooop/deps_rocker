from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Gemini(SimpleRockerExtension):
    """Install Google Gemini CLI tool"""

    name = "gemini"
    depends_on_extension = ("npm",)
