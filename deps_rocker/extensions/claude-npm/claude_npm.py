from deps_rocker.extensions.claude_base import ClaudeBase


class ClaudeNpm(ClaudeBase):
    """Install Claude Code via npm and mount host ~/.claude into the container"""

    name = "claude-npm"
    depends_on_extension: tuple[str, ...] = ("npm", "user", "uv")
