# Use Pixi Builder Pattern for Cargo Extension

## Goal
Update cargo extension to use the pixi builder pattern (like npm) instead of curl-based rustup installation.

## Current Implementation
- Downloads rustup-init.sh via curl
- Installs Rust toolchain with rustup
- Copies `.cargo` and `.rustup` directories from builder
- Depends on curl extension

## New Implementation
Use pixi to install rust/cargo from conda-forge:
- **Builder stage**: Use official pixi image to install rust via `pixi global install rust`
- **System stage**: Copy rust toolchain to `/opt/rust`, create symlinks
- **User stage**: Configure user environment (if needed)
- Remove curl dependency (only used in builder)

## Benefits
- ✅ Consistent with npm extension pattern
- ✅ Use official pixi image (fast, cached)
- ✅ No curl download step
- ✅ Simpler installation
- ✅ Rust from conda-forge (tested, reproducible)

## Additional Work
Add a guide to CLAUDE.md for LLMs on when and how to use the pixi builder pattern for new extensions.
