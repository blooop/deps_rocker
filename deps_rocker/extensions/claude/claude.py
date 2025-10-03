from deps_rocker.extensions.claude_base import ClaudeBase


class Claude(ClaudeBase):
    """Install Claude Code via official installer and mount host ~/.claude into the container"""

    name = "claude"
    depends_on_extension: tuple[str, ...] = ("curl", "user", "uv")
