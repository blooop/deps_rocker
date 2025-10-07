# Docker-in-Docker Debugging Guide

## If the container hangs during startup:

### 1. Interrupt and debug:
```bash
# Press Ctrl+C to interrupt, then run:
docker ps  # Find the hanging container
docker exec -it <container_id> /bin/bash
```

### 2. Check Docker daemon status:
```bash
# Inside the container:
ps aux | grep dockerd
cat /tmp/dockerd.log
tail -f /tmp/dockerd.log
```

### 3. Manual daemon start for debugging:
```bash
# Stop any existing daemon:
pkill dockerd

# Start manually with verbose output:
dockerd --debug

# In another terminal/session:
docker info
docker ps
```

### 4. Check for conflicts:
```bash
# Check volume mounts:
mount | grep docker
df -h | grep docker

# Check processes:
ps aux | grep docker
```

### 5. Quick test without complex setup:
```bash
# Test with minimal setup:
rocker --docker-in-docker ubuntu:22.04

# If that works, gradually add other extensions:
rocker --docker-in-docker --user ubuntu:22.04
rocker --docker-in-docker --user --x11 ubuntu:22.04
# etc.
```
