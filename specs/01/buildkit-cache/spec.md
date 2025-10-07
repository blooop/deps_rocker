# BuildKit Extension Caching Spec

## Goal
Enable extensions to leverage BuildKit's `cache-from` and `cache-to` features so that containers can be rebuilt quickly, sharing cached layers across different extension combinations.

## Key Points
- Each extension should define cacheable build steps (e.g., downloads, installs).
- Use BuildKit cache mounts (`--mount=type=cache`) in Dockerfile snippets for extension steps.
- Support both local and remote cache sources (`cache-from`, `cache-to`).
- Allow cache sharing between builds with different extension sets (deduplicate common steps).
- Extensions should name cache mounts and artifacts with versioned keys to avoid stale cache issues.
- Document best practices for cache usage in extension authoring.

## Out of Scope
- Full implementation of cache server or remote cache infrastructure.
- Non-BuildKit Docker backends.

## Example
- Extension Dockerfile snippet:
  ```Dockerfile
  RUN --mount=type=cache,target=/root/.cache/pip pip install -r requirements.txt
  ```
- Compose build: `docker buildx build --cache-from=type=local,src=./.buildx-cache --cache-to=type=local,dest=./.buildx-cache`

## Open Questions
- How to coordinate cache keys across extensions?
- How to handle cache invalidation for extension upgrades?
