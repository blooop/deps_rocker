# NPM Extension: Install via Pixi

Update the npm extension to install Node.js and npm using pixi with the nodejs package from conda-forge instead of using NVM (Node Version Manager).

## Requirements
- Install Node.js and npm via pixi using the `nodejs` conda-forge package
- Remove dependency on curl and NVM
- Use BuildKit cache mounts for pixi cache (inherited from pixi extension)
- Maintain backward compatibility for users of the npm extension
- Existing tests should continue to work

## Out of Scope
- Changing npm version pinning behavior (pixi manages the version)
- Modifying npm configuration beyond what's needed for pixi installation

## Implementation
- Updated `npm.py` to depend on `pixi` instead of `curl`
- Removed `npm_builder_snippet.Dockerfile` (no longer needed)
- Created `npm_user_snippet.Dockerfile` to use `pixi global install nodejs` in user stage
- Kept NPM environment variables for disabling update notifier and fund messages

## Fix
- Renamed `npm_snippet.Dockerfile` to `npm_user_snippet.Dockerfile` because pixi global install must run in the user stage after pixi is set up in the user environment
