# ROS Repos Underlay Implementation Plan

## Technical Architecture

### Extension Design
- Create `ros_repos_underlay` extension inheriting from `SimpleRockerExtension`
- Depends on: `vcstool`, `ros_generic` (or specific ROS distro), `git`
- Name: `ros_repos_underlay`

### Core Components

#### 1. Repository Discovery
- Scan workspace for `*.repos` files using glob patterns
- Support recursive scanning in subdirectories
- Parse repos files to understand dependencies

#### 2. Dependency Resolution
- Build dependency graph from repos files
- Handle circular dependencies gracefully
- Topological sort for build order

#### 3. Repository Management
- Use vcstool to clone missing repositories
- Check if repositories already exist to avoid re-cloning
- Handle different VCS types (git, hg, svn)

#### 4. Workspace Building
- Create separate underlay workspaces for each repos file
- Use `colcon build` to build each underlay
- Configure build isolation and install spaces

#### 5. Environment Setup
- Source built workspaces in correct order
- Set up CMAKE_PREFIX_PATH and other ROS environment variables
- Ensure main workspace can find all underlay packages

## Implementation Steps

### Phase 1: Basic Structure
- Create extension directory and files
- Set up basic dependency framework
- Implement repository discovery

### Phase 2: Repository Operations
- Integrate with vcstool for cloning
- Parse repos files and extract dependency information
- Handle existing repositories

### Phase 3: Build System Integration
- Implement workspace building with colcon
- Set up proper isolation and install paths
- Handle build failures gracefully

### Phase 4: Environment Configuration
- Create sourcing scripts for underlays
- Set up environment variables correctly
- Ensure proper overlay/underlay chain

### Phase 5: Testing and Validation
- Create comprehensive tests
- Test with sample repos files
- Validate environment setup

## Docker Integration

### Dockerfile Snippet Structure
```dockerfile
# Install build dependencies
RUN apt-get update && apt-get install -y \\
    python3-colcon-common-extensions \\
    && rm -rf /var/lib/apt/lists/*

# Set up workspace directories
RUN mkdir -p /opt/ros/underlays

# Copy and execute setup script
COPY ros_repos_underlay_setup.sh /tmp/
RUN chmod +x /tmp/ros_repos_underlay_setup.sh && \\
    /tmp/ros_repos_underlay_setup.sh

# Source setup in bashrc
RUN echo "source /opt/ros/underlays/setup.bash" >> /etc/bash.bashrc
```

### Setup Script Responsibilities
- Scan for repos files in mounted workspace
- Clone missing repositories to `/opt/ros/src`
- Build repositories to `/opt/ros/build` and `/opt/ros/install`
- Create master setup script that sources all underlays

## Error Handling
- Graceful degradation when repos files are missing
- Skip broken repositories and log warnings
- Continue with partial builds when some dependencies fail

## Caching Strategy
- Use BuildKit cache mounts for repository clones
- Cache built packages when possible
- Invalidate cache when repos files change

## Configuration Options
- Allow specifying custom repos file patterns
- Support for different build types (Debug, Release)
- Option to skip building and only clone repositories

## Testing Strategy
- Unit tests for repository discovery and parsing
- Integration tests with sample ROS packages
- Docker build tests to verify environment setup
- Test recursive dependency resolution
