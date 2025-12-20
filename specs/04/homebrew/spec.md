# homebrew extension spec

- The homebrew extension must install the Homebrew package manager (`brew`) in the container.
- After installation, running `brew --version` should work from any shell (i.e., `brew` must be in `$PATH`).
- The test script should verify that `brew` is available and functional.
- Installation should follow Docker best practices and use cache mounts for downloads.
- Homebrew should be installed for the root user in `/root/.linuxbrew` or `/home/linuxbrew/.linuxbrew`.
- Required dependencies (curl, git, build-essential) should be automatically installed.
