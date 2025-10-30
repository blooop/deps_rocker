# CWD Extension Update Specification

## Purpose
Update the `cwd` extension to mount the current working directory (CWD) inside the container's home directory instead of at the filesystem root.

## Previous Behavior
- Mounted host CWD to `/project_name` at filesystem root
- Set working directory to `/project_name`

## New Behavior
- Mount host CWD inside container's home directory at `~/project_name`
- Set working directory to `~/project_name` in the container
- Preserves the container's home directory structure while adding the project
- No additional software installation required (pure volume mount)

## Example
If host is in `/host/path/projectA`, the container will:
- Mount it to `~/projectA`
- Start in `~/projectA` as the working directory

## Implementation Changes
- Add dependency on `user` extension to get home directory path
- Update `get_docker_args()` to mount at `~/project_name` instead of `/project_name`
- Add imports for `os`, `pwd`, and `logging`
- Set working directory to `~/project_name`
- No Dockerfile snippet needed (no installation required)

## Rationale
Mounting inside home directory is more intuitive and provides better integration with development tools that expect standard home directory structure.
