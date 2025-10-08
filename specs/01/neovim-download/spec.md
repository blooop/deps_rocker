## Goal
- Fetch the pinned Neovim linux64 tarball reliably and error out if the version is missing.
- Make builder stage aliases unique per extension instance to eliminate duplicate-stage warnings.
- Re-declare missing args and switch to `ENV key=value` format to satisfy Dockerfile lint checks.
