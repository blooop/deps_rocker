# Use Pixi for Tool Installation

## Goal
Replace git clone-based tool installations with pixi where the tool is available in conda-forge to simplify extensions and leverage pixi's ecosystem.

## Tools Migrated
- fzf (was git clone + custom install, now pixi global install)

## Tools NOT Migrated
- lazygit: not available in conda-forge/pixi
- nvim: not available in conda-forge/pixi
- gitui: extension doesn't exist

## Changes
- Removed fzf builder stage
- Replaced git clone + install logic with `pixi global install fzf` in user snippet
- Updated test script to manually export PATH for non-interactive shells
- Maintained shell integration setup for key bindings and completion

## Benefits
- Simpler Dockerfile for fzf (no multi-stage build needed)
- Consistent with pixi installation pattern
- Automatic version management via conda-forge
- Faster builds through pixi caching
