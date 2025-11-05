# Auto-Number Specifications

## Goal
Update AGENTS.md to automatically number specifications and group them by functionality.

## Requirements
- Specs should be numbered automatically based on the highest existing number
- All specs should be in `specs/` folder with numbered prefixes: `specs/NN-spec-name/`
- When creating a new spec, find the highest existing number and increment by 1
- Group related specs by functionality categories
- Before creating a new spec, check if it fits into an existing category/group
- Update workflow instructions in AGENTS.md

## Categories
- **cache-optimization**: BuildKit caching, pip cache, vcstool cache, etc.
- **auto-detection**: Auto-extension detection, workspace scanning
- **ros-integration**: ROS Jazzy, underlay, vcstool dependencies
- **tool-extensions**: Individual tool installations (nvim, npm, uv, etc.)
- **build-optimization**: Multi-stage builds, Docker optimization
- **bug-fixes**: Specific bug fixes and patches
- **workflow**: Meta-improvements to development workflow

## Implementation
- Update AGENTS.md workflow section to include numbering logic
- Add category reference for agents to check when creating new specs
- Provide examples of how to determine category membership

## Note
This change only affects AGENTS.md documentation and does not modify any code. CI tests would verify that no code functionality is broken, but since this is a documentation-only change, the risk is minimal.
