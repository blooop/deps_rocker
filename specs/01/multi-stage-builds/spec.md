# Multi-stage BuildKit support

- Introduce builder + final stages in generated Docker snippets so heavy downloads run in disposable layers.
- Reuse BuildKit cache or remote ADD to prevent repeated downloads while allowing parallel fetches.
- Ensure extensions compose the stages cleanly without leaking build-only assets into the runtime image.
- Keep existing extension interface, docs, and tests passing.
