# Palanteer Extension Spec

## Goal
Install the Palanteer profiler (C++/Python) in the container using a builder stage, copying the built binaries to the final image. Ensure the extension works with the deps_rocker build system and passes its test script.

## Key Requirements
- Use a builder stage to build Palanteer from source
- Copy built binaries to /usr/local/bin in the final image
- Provide all required apt dependencies
- Ensure the Python module is importable and the CLI is available
- Pass the extension's test.sh script

## Implementation Notes
- Use empy_args/empy_builder_args to ensure builder stage is included
- Follow patterns from fzf/lazygit extensions
- Use bash -c for RUN commands needing pipefail
- Test with `pixi run pytest -k test_palanteer_extension`
