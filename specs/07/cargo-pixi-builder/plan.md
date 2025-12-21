# Implementation Plan

## 1. Update Cargo Extension Files

### cargo_builder_snippet.Dockerfile
- Use official pixi image: `FROM ghcr.io/prefix-dev/pixi:latest AS cargo-builder`
- Run `pixi global install rust` to install rust/cargo
- Copy rust environment from `/root/.pixi/envs/rust` to builder output

### cargo_snippet.Dockerfile (system stage)
- Copy rust from builder to `/opt/rust`
- Run `chmod -R a+rX /opt/rust` as root for permissions
- Create symlinks in `/usr/local/bin` for cargo, rustc, rustup, etc.

### cargo.py
- Remove curl dependency
- Change to `depends_on_extension = ("user",)`

## 2. Add Pixi Builder Pattern Guide

Add to CLAUDE.md:
- When to use pixi builder pattern
- Template/pattern to follow
- Benefits and trade-offs
- Example implementations (npm, cargo)

## 3. Testing
- Run cargo extension test
- Run full CI
- Verify cargo works during build and at runtime

## 4. Commit
- Update spec if needed
- Commit all changes when CI passes
