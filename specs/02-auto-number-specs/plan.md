# Implementation Plan: Auto-Number Specifications

## Analysis
Currently all specs are in `specs/01/` subdirectories. The new pattern will be `specs/NN-spec-name/` where NN auto-increments.

### Existing Spec Categories

**Cache Optimization (cache-optimization)**
- buildkit-cache-mount
- ccache-shared
- pip-buildkit-cache
- pixi-cache
- vcstool-repo-cache

**Auto-Detection (auto-detection)**
- auto
- auto-detect-llm-extensions
- auto-search-root
- pixi-auto-detect

**ROS Integration (ros-integration)**
- ros
- ros-jazzy-dep-fix
- ros-jazzy-test-fix
- ros-jazzy-unified
- ros-underlay-builder
- ros-workspace-layout
- vcstool-depends-consolidation

**Tool Extensions (tool-extensions)**
- deps-dev
- jquery
- nvim
- uv

**Build Optimization (build-optimization)**
- multi-stage-builds

**Bug Fixes (bug-fixes)**
- neovim-version-pin
- palanteer-dockerfile-copy-fix

**Workflow (workflow)**
- This spec (auto-number-specs)

## Changes to AGENTS.md

### Current Workflow (line 6-9)
```
* On first message:
    - create a new specification according to the pattern specs/01/short-spec-name/spec.md.  Keep it as concise as possible
    - create a plan in the same folder, you can expand more here
    - commit the contents of this folder only
```

### New Workflow
```
* On first message:
    - analyze the request and determine which category it belongs to (cache-optimization, auto-detection, ros-integration, tool-extensions, build-optimization, bug-fixes, workflow)
    - find the highest existing spec number by listing specs/ directory
    - create a new specification at specs/NN-short-spec-name/spec.md where NN is highest+1, zero-padded to 2 digits
    - keep spec.md as concise as possible
    - create a plan.md in the same folder with expanded details
    - commit the contents of this folder only
```

### Add Category Reference Section
Add a new section to AGENTS.md listing the spec categories and examples to help agents determine category membership.

## Steps
1. Update AGENTS.md workflow section (lines 6-9)
2. Add spec category reference section
3. Test with `pixi run ci`
4. Commit if tests pass
