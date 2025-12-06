# Use Pixi for Tool Installation

## Goal
Replace apt/source-based tool installations with pixi where possible to simplify extensions and leverage pixi's conda-forge ecosystem.

## Tools to Migrate
- fzf (currently git clone + install)
- lazygit (currently GitHub release download)
- nvim (currently GitHub release download)
- gitui (if exists - check)

## Changes
- Remove builder stages from migrated extensions
- Replace complex download/build logic with `pixi global install`
- Maintain existing test scripts
- Keep same docker args functionality where applicable (e.g., nvim config mounting)

## Benefits
- Simpler Dockerfiles (no multi-stage builds needed)
- Consistent installation method across tools
- Version management via pixi
- Automatic binary caching through pixi
