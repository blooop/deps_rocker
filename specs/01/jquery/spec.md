# jQuery Extension Spec

Goal: Provide a simple extension named `jquery` which installs the `jq` command-line JSON processor via apt for use in other image build steps. (Name chosen per request even though it provides `jq`).

Scope:
- New rocker extension `jquery` depending on no other extensions.
- Installs apt package `jq` using non-interactive, minimal layer pattern.
- Adds entry point so `--jquery` flag works.
- Provides test script verifying `jq` presence and basic functionality.

Non-goals: Managing JavaScript jQuery library, version pinning for jq beyond apt default.

Outputs: extension code, Docker snippet, test.sh, updated pyproject entry point, test list inclusion.
