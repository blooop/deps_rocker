# Foxglove libasound dependency

- Ubuntu 24.04 no longer ships a concrete `libasound2` package; the foxglove snippet must install the `libasound2t64` provider instead.
- Ensure the snippet stays BuildKit-friendly so cached mounts and CI builds keep working.
- Validate the fix with `pixi r test-extension foxglove` and the full `pixi run ci`.
