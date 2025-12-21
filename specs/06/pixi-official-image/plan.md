# Implementation Plan

## 1. Update Pixi Extension

### pixi_builder_snippet.Dockerfile
- Change FROM to use `ghcr.io/prefix-dev/pixi:latest`
- Copy `/usr/local/bin/pixi` from official image to builder output directory
- Remove curl-based installation logic

### pixi_snippet.Dockerfile
- Keep existing COPY from builder stage (already correct)

### pixi_user_snippet.Dockerfile
- No changes needed (PATH and completion setup remains the same)

## 2. Update NPM Extension

### Add npm.py changes
- Add `builder_apt_packages` if needed
- Configure builder stage to use official pixi image

### Create npm_builder_snippet.Dockerfile (new file)
- Use official pixi image as base: `FROM ghcr.io/prefix-dev/pixi:latest`
- Run `pixi global install nodejs`
- Copy nodejs environment to builder output directory

### Update npm_user_snippet.Dockerfile
- Replace pixi global install with COPY from builder stage
- Copy nodejs binaries from builder to /usr/local/bin or appropriate location
- Remove pixi dependency (pixi won't be needed at runtime)
- Keep PATH and npm config settings

## 3. Testing
- Run `pixi run ci` after changes
- Verify pixi extension still works
- Verify npm extension installs and works without runtime pixi dependency
- Fix any errors that arise

## 4. Commit
- Update spec.md if clarifications needed
- Commit changes once CI passes
