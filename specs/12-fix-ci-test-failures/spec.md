# Fix CI Test Failures

## Problem
Two tests failing in CI but passing locally:
1. `test_claude_integration.py::TestClaudeIntegration::test_claude_config_mount_and_env`
2. `test_extensions_generic.py::TestExtensionsGeneric::test_z_all_extensions_together`

## Root Cause
CI runs in a devcontainer environment with different user context and environment compared to local execution. Tests likely fail due to:
- HOME directory differences
- User/permissions differences
- Docker-in-Docker environment specifics

## Solution
Investigate and fix environment-specific issues in both tests to ensure they work in both local and CI environments.
