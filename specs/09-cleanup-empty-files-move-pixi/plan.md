# Implementation Plan

## Phase 1: Move pixi installs to main snippets

For each extension, we'll:
1. Read the current user snippet to understand what needs to be moved
2. Create or update the main snippet with pixi install
3. Update or remove the user snippet based on remaining content

### gitui
- Create `gitui_snippet.Dockerfile` with `pixi global install gitui`
- Remove `gitui_user_snippet.Dockerfile` (will be empty)

### fzf
- Update `fzf_snippet.Dockerfile` with `pixi global install fzf`
- Keep user snippet with shell integration and cdfzf function (user-specific)

### lazygit
- Update `lazygit_snippet.Dockerfile` with `pixi global install lazygit`
- Keep user snippet with `lg` alias (user-specific)

### deps_devtools
- Update `deps_devtools_snippet.Dockerfile` with `pixi global install ripgrep fd-find`
- Remove `deps_devtools_user_snippet.Dockerfile` (will be empty)

### jquery
- Update `jquery_snippet.Dockerfile` with `pixi global install jq`
- Remove `jquery_user_snippet.Dockerfile` (will be empty)

### ccache
- Update `ccache_snippet.Dockerfile` with `pixi global install ccache` and ENV CCACHE_DIR
- Remove `ccache_user_snippet.Dockerfile` (will be empty)
- Remove outdated apt comment

### curl
- Create `curl_snippet.Dockerfile` with `pixi global install curl`
- Remove `curl_user_snippet.Dockerfile` (will be empty)

### ssh_client
- Create `ssh_client_snippet.Dockerfile` with `pixi global install openssh`
- Remove `ssh_client_user_snippet.Dockerfile` (will be empty)

## Phase 2: Verify extensions still work

Run `pixi run ci` to ensure all tests pass after the changes.

## Phase 3: Clean up any other empty files

Look for other nearly empty files that may have been missed.

## Notes

- Main snippets install packages system-wide (available to all users)
- User snippets are for user-specific configurations (aliases, shell integration, etc.)
- This follows Docker best practices: install in main image, configure in user layer
