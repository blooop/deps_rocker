# Plan

1. Catalog all builder extensions, their current `builder_apt_packages`, and required tooling; map each dependency to pixi packages where available.
2. Design or update a shared builder pattern that installs `builder_pixi_packages` via pixi, keeps `builder_apt_packages` as a fallback hook, and trims repeated boilerplate.
3. Refactor each builder extension to use the pixi-centric pattern, shifting dependencies from apt to pixi and keeping stages concise without breaking behavior.
4. Update supporting docs/tests for the new builder flow and validate with `pixi run ci`, addressing any failures that emerge.
