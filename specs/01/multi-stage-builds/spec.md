# Multi-stage BuildKit support

- Extend snippets to declare BuildKit multi-stage builds (`builder` + `final`) enabling parallel downloads while isolating build artifacts.
- Keep download caching via `--mount=type=cache` or `ADD/COPY` from remote Git commits so repeated builds reuse data.
- Ensure extension composition copies only runtime payloads into the final stage and drops build-only layers.
- Preserve current extension APIs, documentation, and tests.
