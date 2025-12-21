# Fix pixi globals and cache footprint
Category: build-optimization

- Ensure pixi global installs are additive so tools like git, jq, npm remain available after all installs run.
- Rework pixi caching/layering to cut CI disk usage and speed builds by reusing cache without bloating final images.
- Validate the resulting image exposes expected tools and CI completes without running out of space.
