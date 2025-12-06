# Implementation Plan

## 1. Convert curl extension to pixi

- Remove `apt_packages = ["curl", "ca-certificates"]`
- Add pixi-based snippet to install curl globally
- Update test to verify curl works

## 2. Convert ssh_client extension to pixi

- Replace `apt_packages = ["openssh-client"]` with pixi
- Use openssh package from conda-forge
- Verify SSH client functionality

## 3. Update palanteer builder to use pixi compilers

- Replace `build-essential` with pixi `c-compiler`, `cxx-compiler`, `make`
- Replace `python3-dev` with pixi `python` (includes headers)
- Replace X11 dev packages with pixi `xorg-*` packages
- Keep runtime apt_packages for system libraries

## 4. Update isaac_sim to use pixi for build tools

- Move `cmake`, `build-essential`, `python3-pip` to pixi
- Keep system libraries in apt (libglib2.0-0, libglu1-mesa, libxmu-dev)

## 5. Testing

- Run full CI suite
- Verify each modified extension works correctly
- Check that builds are faster without apt operations
