# Plan: Fix palanteer Dockerfile COPY Command

1. Document the problem and solution in a concise spec (done).
2. Update `palanteer_snippet.Dockerfile` to use the correct COPY syntax for binaries from the builder stage.
3. Run `pixi run ci` to verify the fix.
4. If CI passes, commit the changes in the spec and Dockerfile.
