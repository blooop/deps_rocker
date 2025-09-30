# Workdir Extension

Add a `workdir` extension that sets the working directory (WORKDIR) in the Docker container.

## Requirements
- Accept a path argument via `--workdir <path>`
- Generate `WORKDIR <path>` directive in Dockerfile
- Path should be absolute or relative to container filesystem
