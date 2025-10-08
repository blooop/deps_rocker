# Neovim Extension Implementation Plan

1. Create extension directory: deps_rocker/extensions/neovim/
2. Implement Neovim extension class in neovim.py
   - Inherit from SimpleRockerExtension
   - Set name, docstring, and config mounting logic
3. Write neovim_snippet.Dockerfile
   - Download latest Neovim AppImage or tarball
   - Install to /usr/local/bin
   - Clean up cache, use BuildKit cache mount for downloads
   - Add logic to mount ~/.config/nvim and ~/.vim from host
4. Add test.sh script
   - Check neovim is installed and $PATH
   - Check config directories are mounted
   - Run nvim --version
5. Update pyproject.toml entry points
6. Add to EXTENSIONS_TO_TEST in test/test_extensions_generic.py
7. Add test method for neovim extension
8. Commit only the contents of specs/01/neovim/
