# nvim extension spec

- The nvim extension must install the Neovim binary (`nvim`) in the container.
- After installation, running `nvim --version` should work from any shell (i.e., `nvim` must be in `$PATH`).
- The test script should verify that `nvim` is available and functional.
- Use the latest stable release of Neovim unless otherwise specified.
- Installation should follow Docker best practices and use cache mounts for downloads.
