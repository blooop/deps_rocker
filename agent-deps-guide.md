# deps_rocker Agent Configuration Guide

**Purpose**: This guide helps AI agents assist users in creating proper `rockerc.yaml` and `project_name.deps.yaml` configuration files for the deps_rocker system.

**Repository**: https://github.com/blooop/deps_rocker

---

## What is deps_rocker?

deps_rocker is a rocker extension system that automates development dependency installation in Docker containers. It provides:
- **Extensions**: Modular components that add tools/environments to Docker containers
- **rockerc.yaml**: Container runtime configuration (which extensions to enable)
- **.deps.yaml**: Dependency definitions (apt packages, pip packages, scripts, etc.)
- **Auto-detection**: Automatically enables extensions based on project files

---

## Available Extensions

### Package Managers
- **`pixi`** - Cross-platform conda-based package manager (auto-detects: `pixi.toml`, `pyproject.toml` with `[tool.pixi]`)
- **`uv`** - Fast Python package installer and resolver
- **`npm`** - Node.js package manager via nvm (auto-detects: `package.json`, `package-lock.json`, `.npmrc`)
- **`cargo`** - Rust package manager (auto-detects: `Cargo.toml`, `Cargo.lock`)
- **`conda`** - Miniconda for Python environments

### Development Tools
- **`nvim`** - Neovim editor with config mounting
- **`fzf`** - Fuzzy finder for command line
- **`lazygit`** - Terminal UI for git commands
- **`gitui`** - Fast git client (via pixi)
- **`ccache`** - C/C++ compiler cache
- **`palanteer`** - Performance profiling tool

### Host Integration
- **`cwd`** - Mount current working directory into container
- **`cwd_name`** - Name container after current directory
- **`ssh_client`** - SSH client tools
- **`detach`** - Run container in detached/background mode

### Utilities
- **`curl`** - HTTP client with CA certificates
- **`locales`** - System locale configuration (UTF-8 support)
- **`tzdata`** - Timezone data
- **`git_clone`** - Git repository cloning utilities
- **`deps_devtools`** - Development search/analysis tools
- **`auto`** - Auto-detect and enable extensions based on project files

### Robotics
- **`ros_jazzy`** - ROS 2 Jazzy Jalopy distribution
- **`isaac_sim`** - NVIDIA Isaac Sim simulator
- **`urdf_viz`** - URDF visualization tools

### AI/Code Tools
- **`claude`** - Claude AI CLI with config mounting (auto-detects: `.claude/`)
- **`codex`** - OpenAI Codex code generation
- **`gemini`** - Google Gemini AI CLI
- **`spec_kit`** - AI-powered specification toolkit

### Other
- **`jquery`** - JavaScript library
- **`git`** - Git version control

---

## Creating rockerc.yaml

**Purpose**: Define which Docker image to use and which extensions to enable.

**Location**: Project root directory

**Format**:
```yaml
image: <base-docker-image>
args:
  - <extension1>
  - <extension2>
  - <extension3>
```

### Common Templates

#### Python Project (with pixi)
```yaml
image: ubuntu:22.04
args:
  - user          # Create matching user in container
  - pull          # Always pull latest base image
  - cwd           # Mount current directory
  - pixi          # Pixi package manager
  - git           # Git version control
  - nvim          # Neovim editor
```

#### Python Project (with uv)
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - uv            # Fast Python package installer
  - git
```

#### Node.js Project
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - npm           # Node package manager
  - git
  - nvim
```

#### Rust Project
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - cargo         # Rust package manager
  - git
```

#### ROS 2 Project
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - ros_jazzy     # ROS 2 Jazzy
  - git
```

#### Multi-Language Project
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - pixi          # For Python
  - npm           # For Node.js
  - cargo         # For Rust
  - git
  - nvim
  - lazygit       # Git TUI
```

#### Auto-Detection
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - auto          # Automatically detect extensions based on project files
  - git
```

### Important Notes
- Extensions are loaded in the order specified
- Some extensions have dependencies (e.g., `claude` requires `curl` and `user`)
- Dependencies are automatically included, no need to specify them
- Use `auto` extension to automatically enable based on project files detected

---

## Creating .deps.yaml Files

**Purpose**: Define system dependencies (apt packages, pip packages, scripts) that should be installed in the Docker image.

**Naming Convention**: `<project_name>.deps.yaml` or any file matching `*.deps.yaml`

**Location**: Anywhere in project tree (recursively discovered and merged)

**Format**: `{command}_{label}` sections where:
- **command**: `apt`, `pip`, `env`, `script`, `preamble`
- **label**: Grouping name (e.g., `language-toolchain`, `version-control`)

### Template Structure

```yaml
# Environment variables (set in Dockerfile)
env_<label>:
  - VAR_NAME=value
  - ANOTHER_VAR=value

# Apt packages (installed via apt-get)
apt_<label>:
  - package1
  - package2

# Pip packages (installed via pip)
pip_<label>:
  - package1
  - package2

# Custom scripts (relative paths to .deps.yaml location)
script_<label>:
  - ./scripts/install_custom_tool.sh

# Preamble (Docker commands before main layers)
preamble_<label>:
  - RUN custom docker command
```

### Common Examples

#### Basic Python Development
```yaml
# project_name.deps.yaml
env_base:
  - PYTHONUNBUFFERED=1

apt_language-toolchain:
  - python3
  - python3-pip
  - python3-venv

apt_build-tools:
  - build-essential
  - cmake

pip_dev-tools:
  - pytest
  - black
  - ruff
```

#### C++ Development
```yaml
# project_name.deps.yaml
apt_language-toolchain:
  - g++
  - gcc
  - make
  - cmake

apt_build-tools:
  - ninja-build
  - ccache

apt_libraries:
  - libboost-all-dev
  - libeigen3-dev
```

#### Node.js Development
```yaml
# project_name.deps.yaml
apt_version-control:
  - git
  - git-lfs

env_nodejs:
  - NODE_ENV=development
```

#### ROS 2 Dependencies
```yaml
# project_name.deps.yaml
apt_ros-deps:
  - ros-jazzy-rviz2
  - ros-jazzy-nav2-bringup
  - ros-jazzy-slam-toolbox

apt_build-tools:
  - python3-colcon-common-extensions
  - python3-rosdep
```

#### Multi-Layer Example
```yaml
# project_name.deps.yaml

# Layer 1: Base utilities (cached, rarely changes)
apt_software-sources:
  - wget
  - curl
  - ca-certificates

# Layer 2: Version control (changes occasionally)
apt_version-control:
  - git
  - git-lfs
  - mercurial

# Layer 3: Language toolchain (changes occasionally)
apt_language-toolchain:
  - python3-pip
  - gcc
  - make

# Layer 4: Development libraries (changes more often)
apt_dev-libraries:
  - libssl-dev
  - libffi-dev
  - libxml2-dev

# Layer 5: Python packages (changes frequently)
pip_language-toolchain:
  - numpy
  - pandas
  - matplotlib

# Environment variables
env_base:
  - WORKSPACE=/workspace
  - PYTHONPATH=/workspace/src
```

#### Custom Install Script Example
```yaml
# project_name.deps.yaml

apt_prerequisites:
  - wget
  - tar

script_custom-tools:
  - ./scripts/install_custom_binary.sh
```

Example `scripts/install_custom_binary.sh`:
```bash
#!/bin/bash
set -e

wget https://example.com/tool.tar.gz
tar -xzf tool.tar.gz
mv tool /usr/local/bin/
chmod +x /usr/local/bin/tool
rm tool.tar.gz
```

---

## Decision Tree for Agents

### When user asks to set up a project, follow this flow:

1. **Identify Project Type**
   - Ask: "What programming language(s) does your project use?"
   - Detect from files if present (package.json, Cargo.toml, pyproject.toml, etc.)

2. **Choose Extensions**
   - Python → `pixi` or `uv`
   - Node.js → `npm`
   - Rust → `cargo`
   - ROS → `ros_jazzy`
   - Unknown/Multiple → `auto`
   - Always consider: `user`, `pull`, `cwd`, `git`

3. **Create rockerc.yaml**
   - Use template for detected language
   - Add common extensions (user, pull, cwd, git)
   - Add requested tools (nvim, lazygit, etc.)

4. **Ask About System Dependencies**
   - "Do you need any system packages installed (apt)?"
   - "Do you need any Python packages (pip)?"
   - "Do you have custom installation scripts?"

5. **Create .deps.yaml (if needed)**
   - If system dependencies needed → create `<project_name>.deps.yaml`
   - Organize by layer (sources, version-control, language, libraries, pip)
   - Add environment variables if specified

6. **Provide Usage Instructions**
   ```bash
   # Build and run container
   rocker .

   # Or with auto-detection
   rocker --auto ubuntu:22.04
   ```

---

## Common Use Cases

### Use Case 1: Python Data Science Project
**User**: "I'm working on a Python data science project with pandas and matplotlib"

**Agent Should Create**:

`rockerc.yaml`:
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - uv
  - git
  - nvim
```

`datascience.deps.yaml`:
```yaml
env_base:
  - PYTHONUNBUFFERED=1

apt_language-toolchain:
  - python3
  - python3-pip

pip_data-science:
  - pandas
  - matplotlib
  - numpy
  - jupyter
```

---

### Use Case 2: ROS 2 Robotics Project
**User**: "I need a ROS 2 Jazzy environment with navigation stack"

**Agent Should Create**:

`rockerc.yaml`:
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - ros_jazzy
  - git
```

`robot.deps.yaml`:
```yaml
apt_ros-packages:
  - ros-jazzy-navigation2
  - ros-jazzy-nav2-bringup
  - ros-jazzy-slam-toolbox

apt_build-tools:
  - python3-colcon-common-extensions
```

---

### Use Case 3: Full-Stack Web Development
**User**: "I'm building a web app with React frontend and Python FastAPI backend"

**Agent Should Create**:

`rockerc.yaml`:
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - npm
  - uv
  - git
  - lazygit
```

`webapp.deps.yaml`:
```yaml
apt_language-toolchain:
  - python3
  - python3-pip

pip_backend:
  - fastapi
  - uvicorn
  - sqlalchemy
  - pydantic
```

---

### Use Case 4: Minimal Project (No System Dependencies)
**User**: "I just need a basic Python environment managed by pixi"

**Agent Should Create**:

`rockerc.yaml`:
```yaml
image: ubuntu:22.04
args:
  - user
  - pull
  - cwd
  - pixi
  - git
```

**No .deps.yaml needed** - pixi manages all dependencies via pixi.toml

---

## Quick Reference: Extension Dependencies

Some extensions automatically include others:

- **`claude`** → requires `curl`, `user`
- **`npm`** → requires `curl`
- **`pixi`** → requires `curl`
- **`uv`** → requires `curl`
- **`lazygit`** → requires `git`, `curl`
- **`gitui`** → requires `pixi`
- **`ros_jazzy`** → requires `locales`, `tzdata`

Don't manually add dependencies; they're automatically included.

---

## Validation Checklist

Before finalizing configuration, verify:

- [ ] `rockerc.yaml` has valid YAML syntax
- [ ] Image specified (e.g., `ubuntu:22.04`)
- [ ] Extensions listed under `args:` as list items
- [ ] Common extensions included: `user`, `pull`, `cwd`, `git`
- [ ] `.deps.yaml` uses correct format: `{command}_{label}`
- [ ] Valid commands: `apt`, `pip`, `env`, `script`, `preamble`
- [ ] Script paths in `script_` sections are relative to .deps.yaml location
- [ ] Environment variables use `VAR=value` format
- [ ] Project-specific naming: `<project_name>.deps.yaml`

---

## Error Prevention

### Common Mistakes to Avoid:

1. **Don't duplicate dependencies**: If `claude` is enabled, don't manually add `curl`
2. **Don't add extensions that aren't in the available list**: Check the "Available Extensions" section
3. **Don't use absolute paths in script_**: Use relative paths from .deps.yaml location
4. **Don't mix commands in same section**: `apt_tools` should only contain apt packages, not pip packages
5. **Don't forget the hyphen**: Items under `args:` must start with `- `

### Good vs Bad Examples:

❌ **Bad**:
```yaml
# Missing hyphen
args:
  user
  cwd
```

✅ **Good**:
```yaml
args:
  - user
  - cwd
```

❌ **Bad**:
```yaml
# Mixing package types
apt_packages:
  - git
  - numpy  # This is a pip package!
```

✅ **Good**:
```yaml
apt_packages:
  - git

pip_packages:
  - numpy
```

---

## Usage Commands

After creating configuration files:

```bash
# Use rockerc.yaml in current directory
rocker .

# Use specific image with auto-detection
rocker --auto ubuntu:22.04

# Use specific extensions via CLI
rocker --pixi --npm --cwd ubuntu:22.04

# Build with dependencies
rocker --deps ubuntu:22.04

# Override rockerc.yaml
rocker --pixi --npm ubuntu:22.04  # Ignores rockerc.yaml
```

---

## Additional Resources

- **Documentation**: https://github.com/blooop/deps_rocker/blob/main/README.md
- **Extension Examples**: https://github.com/blooop/deps_rocker/tree/main/deps_rocker/extensions
- **Test Examples**: https://github.com/blooop/deps_rocker/tree/main/test

---

## Agent Instructions Summary

When a user asks for help setting up deps_rocker configuration:

1. **Detect project type** from files or by asking
2. **Choose appropriate extensions** from the available list
3. **Create `rockerc.yaml`** with selected extensions
4. **Ask about system dependencies** (apt, pip, scripts)
5. **Create `.deps.yaml`** only if system dependencies needed
6. **Organize .deps.yaml by layer** (sources → version-control → language → libraries → pip)
7. **Validate** using the checklist
8. **Provide usage command**: `rocker .`

Always prioritize simplicity: use `auto` extension when unsure, and skip `.deps.yaml` if not needed.
