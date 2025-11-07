# Foxglove Browser Integration

## Problem
When using Foxglove Studio in a container, clicking links doesn't open the browser, preventing proper integration with web-based features.

## Solution
Add browser integration to the foxglove extension by:

1. Adding `--network host` Docker argument to enable network connectivity for browser callbacks
2. Adding `x11` as a dependency to ensure GUI/browser functionality is available
3. Following the same pattern used by the claude extension for browser-based authentication

## Implementation
- Update `foxglove.py` to include `x11` dependency and add `--network host` to Docker arguments
- This will enable browser links to work properly when clicked from within Foxglove Studio

## Acceptance Criteria
- Foxglove extension includes `x11` dependency
- Container runs with `--network host` for browser connectivity
- Links clicked in Foxglove Studio open in the host browser
