# deps-devtools Extension Spec

- Name: deps-devtools
- Installs via apt: ripgrep, fd-find
- Depends on: fzf extension
- Purpose: Provides developer search tools (ripgrep, fd-find, fzf) for enhanced CLI productivity.
- Dockerfile: Installs ripgrep and fd-find using apt, leverages fzf extension for fuzzy finding.
- Entry point: Register in pyproject.toml under rocker.extensions
- Test: Verifies ripgrep, fd-find, and fzf are available and functional
