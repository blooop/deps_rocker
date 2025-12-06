# Replace apt with pixi where possible

Replace apt package installation with pixi/conda-forge packages to significantly speed up builds and avoid slow apt operations.

## Packages to Replace

### Extensions that can fully migrate to pixi:
- **curl**: `curl`, `ca-certificates` → pixi global install curl
- **ssh_client**: `openssh-client` → pixi (openssh package)

### Extensions with partial migration:
- **palanteer**:
  - Builder: `build-essential`, `python3-dev`, X11 dev libs → pixi (c-compiler, cxx-compiler, python, xorg-* packages)
  - Runtime: X11 runtime libs → Keep apt (system libraries needed for GUI)

- **isaac_sim**:
  - `cmake`, `build-essential` → pixi
  - System libs (`libglib2.0-0`, `libglu1-mesa`, `libxmu-dev`) → Keep apt (NVIDIA runtime dependencies)

### Extensions to keep using apt:
- **urdf_viz**: ROS packages (`ros-humble-xacro`) and X11 libs → Keep apt (ROS ecosystem requirement)
- **ros_jazzy**: ROS specific packages → Keep apt

## Strategy

1. Create pixi-based snippets for curl and ssh_client extensions
2. Update builder stages to use pixi for build tools (cmake, compilers)
3. Keep runtime system libraries in apt where required for compatibility
4. Test each extension thoroughly
