# Foxglove Browser Integration Implementation Plan

## Analysis
The claude extension successfully enables browser integration for authentication callbacks using two key components:

1. **Network connectivity**: Uses `--network host` to allow the container to communicate with the host network, enabling browser callbacks
2. **X11 dependency**: Ensures GUI functionality is available for browser-based interactions

## Implementation Steps

### 1. Update Dependencies
- Add `x11` to the `depends_on_extension` tuple in `foxglove.py`
- This ensures that X11 forwarding is set up for GUI applications

### 2. Update Docker Arguments  
- Modify the `get_docker_args()` method to include `--network host`
- This allows the container to access the host network for browser connectivity
- Combine with existing volume mounts for foxglove agent storage

### 3. Code Changes
```python
# In foxglove.py:
depends_on_extension = ("curl", "x11")  # Add x11

def get_docker_args(self, cliargs) -> str:
    # Existing volume mounts
    home_dir = os.path.expanduser("~")
    recordings_dir = os.path.join(home_dir, "foxglove_recordings")
    os.makedirs(recordings_dir, exist_ok=True)
    
    volume_args = f' -v foxglove-agent-index:/index -v "{recordings_dir}:/storage"'
    network_args = " --network host"  # Add network host for browser integration
    
    return volume_args + network_args
```

## Benefits
- Enables browser links to open properly from Foxglove Studio
- Follows established pattern from claude extension
- Maintains all existing functionality (agent storage, recordings)
- Minimal changes required

## Testing
- Verify Foxglove Studio starts correctly
- Test that clicking links opens the host browser
- Ensure agent storage and recordings still work
- Confirm X11 forwarding works for GUI elements
