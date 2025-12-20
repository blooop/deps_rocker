# Fix claude extension under pixi

- Category: bug-fixes
- Problem: claude extension broke after migrating extensions to pixi, causing CI/test failures.
- Scope: align claude extension behavior and tests with main while keeping the pixi workflow.
- Clarification: reintroduce explicit builder dependencies (curl/ca-cert) so claude downloads don’t rely on the base image.
- Deliverable: claude extension builds and tests pass under pixi with minimal test divergence from main.
