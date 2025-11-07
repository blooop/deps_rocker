# Plan

1. Audit `deps_rocker/extensions/foxglove/foxglove_snippet.Dockerfile` (and related templates) for the `libasound2` dependency and any other Ubuntu 24.04 incompatibilities.
2. Replace the virtual `libasound2` entry with the concrete provider `libasound2t64`, keeping the package list formatting and cache mounts intact.
3. Search for additional references to `libasound2` and align them with the Ubuntu 24.04 provider if needed.
4. Run `pixi run ci`; iterate on adjustments until the build and tests succeed.
5. Once CI passes, prepare the change for commit per workflow guidance.
