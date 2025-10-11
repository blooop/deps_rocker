# Plan

1. Reproduce the failure locally by reviewing logs and running the failing portion of `pixi run ci`.
2. Inspect the docker-in-docker extension setup to determine why `docker run` exits non-zero when launched non-interactively.
3. Implement the minimal fix (e.g., ensure daemon startup, adjust entrypoint, or gate behavior) without regressing other extensions.
4. Validate the change with `pixi run ci` and iterate until it passes.
5. Clean up and document any key behavior differences if needed.
