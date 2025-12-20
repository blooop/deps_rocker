# Pixi adoption and CI stabilization (workflow)

Bring this branch in line with main by migrating remaining package installs to pixi and ensuring CI succeeds after the port.

## Actions
1. Review branch vs main to identify extensions/snippets still relying on non-pixi installs and convert them to pixi-managed installs.
2. Align pixi task definitions in `pyproject.toml` with the ported packages so local and CI workflows use pixi consistently.
3. Run `pixi run ci` and iterate on test/build fixes until it passes with the pixi-based setup.
4. Update any related docs or comments to reflect the pixi-based installations.
