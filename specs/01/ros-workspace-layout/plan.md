# Plan

1. Audit current ros_jazzy, vcstool, ros_underlay, and cwd extensions to map where repos and underlays live now.
2. Evaluate candidate directory layouts (root, repo workspace, home) against permission model and UX.
3. Select canonical layout that keeps artifacts writable and discoverable without polluting downstream repos.
4. Design required code changes across extensions (env vars, Docker snippets, helper scripts) to adopt the layout.
5. Outline migration steps (cleanup paths, symlinks, documentation) to ease transition.
