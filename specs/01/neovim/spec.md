# Neovim Extension Spec

Install the latest stable Neovim in the container via PPA: `add-apt-repository ppa:neovim-ppa/stable` (not AppImage or tarball)
Use best practices for Docker layer caching and cleanup
Mount host's ~/.config/nvim and ~/.vim directories into the container for persistent config
Ensure neovim is available in $PATH and works in the container
Add a test script to verify installation and config mounting
Update entry points and tests as per AGENTS.md checklist
