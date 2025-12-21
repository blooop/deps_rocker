# Spec: Pixi Builder Pattern for NPM Extension

## Goal
Convert npm extension to use pixi as a builder stage, copying only the nodejs binaries without including the pixi cache in the final image.

## Requirements
- Use pixi in builder stage to install nodejs via `pixi global install nodejs`
- Copy only the nodejs environment binaries to a shared location (e.g., `/opt/deps_rocker/nodejs`)
- User snippet copies from shared location to user's home directory
- Maintain all existing functionality (node, npm, npx commands available)
- Keep builds fast and cacheable
- Follow the pattern established by the pixi extension

## Success Criteria
- CI passes
- nodejs/npm/npx commands work in built container
- No pixi cache included in final image layers
- Pattern is reusable for other pixi-based extensions
