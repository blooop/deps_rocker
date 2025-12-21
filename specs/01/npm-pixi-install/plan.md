# Implementation Plan: NPM via Pixi

## Changes Required

### 1. Update npm.py
- Remove `depends_on_extension = ("curl",)`
- Add `depends_on_extension = ("pixi",)` instead
- Remove `builder_apt_packages` since pixi handles installation
- Update or remove `empy_args` - pixi will manage the nodejs version
- Keep the class simple since pixi does the heavy lifting

### 2. Update npm_snippet.Dockerfile
- Remove NVM installation logic
- Remove ENV variables for NVM_DIR and NODE_VERSION
- Add pixi installation snippet using `pixi global install nodejs`
- Ensure npm and node binaries are in PATH via pixi
- Use BuildKit cache mount for pixi cache (inherited from pixi extension)

### 3. Test Verification
- Verify test.sh still works and checks for npm and node commands
- Ensure version checks pass
- Test that npm can install packages

## Benefits
- Simpler installation using pixi ecosystem
- Better caching via BuildKit mounts
- Consistent with other pixi-based extensions
- Eliminates need for separate NVM installation
- Easier to manage nodejs versions through pixi
