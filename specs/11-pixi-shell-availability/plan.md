# Plan: Pixi Shell Availability

## Investigation
1. Examine the current pixi extension implementation
2. Check how pixi is currently being installed and configured
3. Identify where shell initialization should be added

## Implementation Steps

### 1. Review Current Pixi Extension
- Read `deps_rocker/extensions/pixi/pixi.py`
- Read `deps_rocker/extensions/pixi/pixi_snippet.Dockerfile`
- Understand current installation method

### 2. Modify Pixi Dockerfile Snippet
- Add pixi initialization to `/etc/bash.bashrc` for bash interactive shells
- Create `/etc/profile.d/pixi.sh` for login shells and other POSIX shells
- Ensure the pixi binary directory is in PATH

### 3. Testing Strategy
- Run `pixi run ci` to ensure all tests pass
- Verify pixi is available immediately in new shells
- Check that the change doesn't break existing functionality

## Technical Details

Pixi installer typically installs to `~/.pixi` and adds initialization to the user's shell config. For system-wide availability in Docker containers, we need to:

1. Install pixi to a system location (or keep user location but make it accessible)
2. Add initialization commands to system-wide shell configs:
   - `/etc/bash.bashrc` - for bash interactive shells
   - `/etc/profile.d/pixi.sh` - for login shells (sh, bash, zsh, etc.)

The initialization typically involves:
- Adding pixi binary to PATH
- Running pixi's shell hook/completion setup if available

## Risks & Considerations
- Ensure this works for both interactive and non-interactive shells
- Don't break existing pixi functionality
- Consider multi-user scenarios (though Docker containers typically run as single user)
