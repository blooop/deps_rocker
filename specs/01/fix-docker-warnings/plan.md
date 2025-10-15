# Plan: Fix Docker Warnings

## Steps

1. **Run all extensions test**
   - Execute the test that uses all extensions together
   - Capture and analyze Docker warnings

2. **Identify warning sources**
   - Parse warning messages
   - Map warnings to specific Dockerfile snippets
   - Categorize by type (apt-get, deprecations, etc.)

3. **Fix warnings**
   - Update affected Dockerfile snippets
   - Follow Docker best practices
   - Common fixes:
     - Add `-y` flags to apt-get commands
     - Use `DEBIAN_FRONTEND=noninteractive`
     - Fix deprecated syntax
     - Add proper cleanup commands

4. **Verify fixes**
   - Re-run all extensions test
   - Confirm no warnings remain
   - Run full CI suite
   - Iterate until CI passes

5. **Commit changes**
   - Commit spec and plan first
   - Commit fixes after CI passes
