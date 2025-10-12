# Plan for Pixi Auto-Detect Extension

1. Update auto-detection logic:
   - Support specifying both filename and content search.
   - Allow content search to be a regex or string.
2. Update pixi extension:
   - When auto-detecting `pyproject.toml`, check for `[tool.pixi]` section.
   - Only activate if section is present.
3. Refactor detection code:
   - Make detection reusable for other extensions.
4. Add/Update tests:
   - Test detection with and without `[tool.pixi]` in `pyproject.toml`.
   - Test generic filename+content search.
5. Document changes in spec and README if needed.
6. Commit only the contents of the spec folder for this step.
