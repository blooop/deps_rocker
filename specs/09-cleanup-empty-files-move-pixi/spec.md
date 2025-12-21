# Cleanup: Remove empty files and move pixi installs to main snippets

After replacing apt with pixi, many snippet files are nearly empty or contain only user-specific configurations. Additionally, several pixi installs are in user snippets when they should be in main Docker snippets.

## Actions

1. **Move pixi installs from user snippets to main snippets**
   - gitui, fzf, lazygit, deps_devtools, jquery, ccache, curl, ssh_client
   - Keep user-specific configurations (shell integration, aliases, env setup) in user snippets
   - Move the actual pixi global install commands to main snippets

2. **Remove nearly empty snippet files**
   - Files containing only `# syntax=docker/dockerfile:1.4` with no actual content
   - Empty files that serve no purpose

3. **Update outdated comments**
   - Replace references to apt packages with pixi where applicable
   - Remove stale comments about installation methods

## Files to modify

### Move pixi installs to main snippets:
- `gitui/gitui_user_snippet.Dockerfile` -> create `gitui/gitui_snippet.Dockerfile`
- `fzf/fzf_user_snippet.Dockerfile` -> `fzf/fzf_snippet.Dockerfile` (keep shell integration in user)
- `lazygit/lazygit_user_snippet.Dockerfile` -> `lazygit/lazygit_snippet.Dockerfile` (keep alias in user)
- `deps_devtools/deps_devtools_user_snippet.Dockerfile` -> `deps_devtools/deps_devtools_snippet.Dockerfile`
- `jquery/jquery_user_snippet.Dockerfile` -> `jquery/jquery_snippet.Dockerfile`
- `ccache/ccache_user_snippet.Dockerfile` -> `ccache/ccache_snippet.Dockerfile`
- `curl/curl_user_snippet.Dockerfile` -> create `curl/curl_snippet.Dockerfile`
- `ssh_client/ssh_client_user_snippet.Dockerfile` -> create `ssh_client/ssh_client_snippet.Dockerfile`

### Remove after moving:
- User snippets that will be empty after moving pixi installs
- Nearly empty main snippets that will be replaced

## Benefits

- Cleaner codebase with fewer unnecessary files
- Consistent pattern: main snippets for installations, user snippets for user-specific config
- Pixi packages installed system-wide, available to all users in the container
- Easier to maintain and understand the extension structure
