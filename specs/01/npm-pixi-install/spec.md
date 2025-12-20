# NPM Extension: Install via Pixi

Update the npm extension to install Node.js and npm using pixi with the nodejs package from conda-forge instead of using NVM (Node Version Manager).

## Requirements
- Install Node.js and npm via pixi using the `nodejs` conda-forge package
- Remove dependency on curl and NVM
- Use BuildKit cache mounts for pixi cache
- Maintain backward compatibility for users of the npm extension
- Update tests to verify npm and node are properly installed

## Out of Scope
- Changing npm version pinning behavior
- Modifying npm configuration beyond what's needed for pixi installation
