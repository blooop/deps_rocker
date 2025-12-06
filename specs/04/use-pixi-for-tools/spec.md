# Use Pixi for Tool Installation

## Goal
Replace apt/git-based tool installations with pixi where the tool is available in conda-forge to simplify extensions and leverage pixi's ecosystem.

## Tools Migrated
- **fzf** (was git clone + custom install, now pixi global install)
- **deps-devtools** (ripgrep, fd-find - was apt install, now pixi global install)
- **lazygit** (was GitHub release download, now pixi global install)
- **ccache** (was apt install, now pixi global install)
- **git_clone** (git, git-lfs - was apt install, now pixi global install)

## Tools NOT Migrated
- **curl**: Cannot be migrated because it's required to bootstrap pixi itself (circular dependency)
- **nvim**: Not available in conda-forge/pixi
- **gitui**: Extension doesn't exist

## Changes

### fzf Extension
- Removed fzf builder stage
- Replaced git clone + install logic with `pixi global install fzf` in user snippet
- Updated test script to manually export PATH for non-interactive shells
- Maintained shell integration setup for key bindings and completion

### deps-devtools Extension
- Removed apt_packages (ripgrep, fd-find)
- Added pixi dependency
- Created user snippet to install ripgrep and fd-find via pixi
- Updated test script to:
  - Export PATH for non-interactive shells
  - Check for `fd` instead of `fdfind` (pixi naming convention)
  - Check for `rg` (ripgrep command)

### lazygit Extension
- Removed GitHub release download builder stage
- Removed curl, git_clone dependencies and builder_apt_packages
- Added pixi dependency
- Created user snippet to install lazygit via pixi
- Updated test script to export PATH for non-interactive shells
- Maintained bash alias (lg='lazygit')

### ccache Extension
- Removed apt_packages (ccache)
- Added pixi dependency
- Created user snippet to install ccache via pixi
- Updated test script to export PATH for non-interactive shells
- Maintained ccache_snippet.Dockerfile for ENV CCACHE_DIR
- Kept get_docker_args for mounting host ccache directory

### git_clone Extension
- Removed apt_packages (git, git-lfs, ca-certificates)
- Added pixi dependency
- Created user snippet to install git and git-lfs via pixi
- Updated test script to:
  - Export PATH for non-interactive shells
  - Test both git and git-lfs versions

## Benefits
- Simpler Dockerfiles (no apt or multi-stage builds)
- Consistent installation method across all dev tools
- Automatic version management via conda-forge
- Faster builds through pixi caching
- Better tool versions from conda-forge vs apt
