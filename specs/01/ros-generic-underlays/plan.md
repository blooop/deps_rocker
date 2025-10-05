# Plan

1. Inspect current `ros_generic` extension to understand how underlays are configured and invoked.
2. Design discovery mechanism for `*.repos` files, including recursion through `depends.repos` manifests.
3. Define how discovered underlays integrate with existing dependency handling and ensure idempotent state.
4. Add entry point (script/alias) in the image to rebuild underlays inside the container using the same logic.
5. Update tests and CI tasks to cover the new behaviour.
