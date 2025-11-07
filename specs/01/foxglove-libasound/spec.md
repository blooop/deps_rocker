# Foxglove libasound dependency

- Ubuntu 24.04 no longer ships a concrete `libasound2` package; the foxglove snippet must install the `libasound2t64` provider instead.
- Ensure the snippet stays BuildKit-friendly so cached mounts and CI builds keep working.
- Validate the fix with `pixi r test-extension foxglove` and the full `pixi run ci`.
- Ensure `test_foxglove_extension` executes (remove or prevent skip) so regressions surface.
- Adopt the shared builder-cache pattern so Foxglove assets download once and are copied into the final image.
- Provide a root-friendly launch wrapper (adds `--no-sandbox`) so `foxglove-studio` starts inside containers without user namespaces.
