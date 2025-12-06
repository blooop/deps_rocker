# Implementation Plan: Use Pixi for Tool Installation

## Overview
Migrate extensions from custom download/build logic to using pixi global install for simpler, more maintainable Dockerfiles.

## Extension Migration Strategy

### For each extension (fzf, lazygit, nvim):

1. **Update Python class**
   - Change `depends_on_extension` to include `pixi` instead of `curl`, `git_clone`, etc.
   - Remove `builder_apt_packages` (no longer need build tools)
   - Remove `empy_args` for versions (pixi handles versioning)

2. **Replace Dockerfile snippets**
   - Delete `*_builder_snippet.Dockerfile` (no multi-stage builds needed)
   - Replace `*_snippet.Dockerfile` content with:
     ```dockerfile
     RUN --mount=type=cache,target=/opt/pixi/envs,id=pixi-envs \
         pixi global install <tool>
     ```
   - For nvim: Keep the volume mounting logic in the Python class

3. **Update or verify test scripts**
   - Ensure tests still validate tool installation
   - No changes needed if tests just check command availability

## Specific Extension Details

### fzf
- Current: Git clone + custom install script with caching
- New: `pixi global install fzf`
- Keep: User snippet for fzf shell integration
- Depends on: pixi, user

### lazygit
- Current: GitHub release download with version detection
- New: `pixi global install lazygit`
- Keep: Bash alias for `lg` command
- Depends on: pixi

### nvim
- Current: GitHub release download with version caching
- New: `pixi global install neovim`
- Keep: Config directory mounting in get_docker_args()
- Depends on: pixi

## Implementation Order
1. fzf (has user snippet to preserve)
2. lazygit (simplest case)
3. nvim (has docker args logic to preserve)

## Verification
- Run tests for each extension after migration
- Verify tools are accessible in resulting containers
- Check that pixi cache is being used (faster rebuilds)
