# Implementation Plan

## 1. Update `_get_builder_pixi_snippet` in `simple_rocker_extension.py`

Replace the current curl-based installation with a multi-stage approach:
- Use `COPY --from=ghcr.io/prefix-dev/pixi:latest /usr/local/bin/pixi /usr/local/bin/pixi`
- Remove all apt-get logic for installing curl/ca-certificates
- Ensure pixi is executable
- Keep the pixi cache mount logic

## 2. Update pixi extension

- Remove `builder_pixi_packages = ["curl", "ca-certificates"]` from `pixi.py`
- Keep `depends_on_extension = ("user",)` since that's still needed

## 3. Test

- Run `pixi run ci` to ensure all tests pass
- Verify that extensions using `builder_pixi_packages` still work correctly
- Confirm no apt packages are needed for pixi installation
