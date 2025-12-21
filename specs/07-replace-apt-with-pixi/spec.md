# Replace apt with pixi where possible

Replace apt package installation with pixi/conda-forge packages to significantly speed up builds and avoid slow apt operations.

## Changes Made

### Extensions fully migrated to pixi:
1. **curl**: Replaced `apt_packages = ["curl", "ca-certificates"]` with pixi global install
   - Added dependency on pixi extension
   - Created user snippet for installation

2. **ssh_client**: Replaced `apt_packages = ["openssh-client"]` with pixi global install
   - Changed to depend on pixi and user
   - Created user snippet for openssh installation

### Extensions with partial migration:

3. **palanteer**: Moved all builder dependencies to pixi
   - Builder: Replaced `build-essential`, `python3-dev`, X11 dev libs with pixi packages
   - Uses: `c-compiler`, `cxx-compiler`, `make`, `python`, mesa/X11 devel packages
   - Runtime: Kept apt packages for system libraries

4. **isaac_sim**: Moved build tools to pixi
   - Builder pixi packages: `cmake`, `c-compiler`, `cxx-compiler`, `make`, `python`, `pip`
   - Kept system libraries in apt for NVIDIA runtime compatibility

### Test improvements:
5. **test_claude_integration.py**: Updated to use RockerExtensionManager
   - Properly resolves transitive dependencies (pixi → curl → claude)
   - Ensures all dependencies are included in build

## Results
- All CI tests passing (76 passed, 13 skipped)
- Significantly faster builds by avoiding apt operations
- Reduced apt usage to only essential system libraries
