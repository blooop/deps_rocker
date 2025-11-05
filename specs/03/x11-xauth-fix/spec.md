# X11 Xauth Missing .Xauthority Fix

## Problem
The rocker X11 extension fails when `~/.Xauthority` doesn't exist on the host system:
```
xauth:  file /home/user/.Xauthority does not exist
xauth: (argv):1:  unable to read any entries from file "(stdin)"
```

## Root Cause
The upstream rocker X11 extension runs `xauth nlist $DISPLAY` which tries to read from `~/.Xauthority`. If this file doesn't exist, xauth fails.

## Solution Options

### Option 1: Documentation (Immediate)
Document the workaround in deps_rocker README:
- Create empty `.Xauthority`: `touch ~/.Xauthority`
- Or remove x11 from rockerc.yaml if GUI not needed

### Option 2: Wrapper Extension (Robust)
Create a `x11_safe` deps_rocker extension that:
1. Checks if `~/.Xauthority` exists
2. Creates it if missing before invoking X11 extension
3. Uses `depends_on_extension = ("x11",)` to ensure proper ordering

### Option 3: Patch Upstream (Long-term)
Submit PR to rocker to handle missing `.Xauthority` gracefully by checking file existence before running `xauth nlist`.

## Recommended Approach
Start with Option 1 (documentation), implement Option 2 if needed by users.
