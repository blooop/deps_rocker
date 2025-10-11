# Multi-stage BuildKit support

- Extend snippets to declare BuildKit multi-stage builds (`builder` + `final`) enabling parallel downloads while isolating build artifacts.
- Keep download caching via `--mount=type=cache` or `ADD/COPY` from remote Git commits so repeated builds reuse data.
- Ensure extension composition copies only runtime payloads into the final stage and drops build-only layers.
- Preserve current extension APIs, documentation, and tests.

# Plan

1. Inspect existing BuildKit cache-mount implementation across templates and extensions to understand current assembly flow.
2. Design a multi-stage snippet pattern (builder stage + final stage) that keeps downloads cached, supports parallel fetch, and composes with extension layering.
3. Update Docker snippet templates and any helper code to emit the new stages without breaking current extension contracts.
4. Adjust affected extensions (and docs/tests) to adopt the pattern, ensuring intermediate artifacts are cleaned from the final image.
5. Run `pixi run ci`, iterate on failures, and commit once the pipeline passes.
