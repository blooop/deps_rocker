# Pixi builder migration (build-optimization)

- Migrate all builder stages to install their toolchain via pixi using `builder_pixi_packages`, keeping `builder_apt_packages` only for unavoidable system deps.
- Introduce a shared builder template or pattern to reduce duplicated builder-stage boilerplate while preserving current behavior.
- Ensure each builder stage remains functional with pixi-provisioned dependencies and update supporting docs/tests as needed.
