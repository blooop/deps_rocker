# Palanteer Extension Fix: Dockerfile COPY Syntax

## Problem
The Dockerfile for the palanteer extension had an invalid COPY command:

```
COPY --from=palanteer_builder/opt/deps_rocker/palanteer/<built-in function bin>/ /usr/local/bin/
```

This caused Docker build errors due to an invalid reference format.

## Solution
- Use the correct Docker COPY syntax to copy binaries from the builder stage.
- Use the correct path: `/opt/deps_rocker/palanteer/bin/` from the `palanteer_builder` stage.
- If using empy templating, ensure f-string syntax is used for template variables.

## Resulting COPY Command
```
COPY --from=palanteer_builder /opt/deps_rocker/palanteer/bin/ /usr/local/bin/
```

## Reference
See EMPY_DEBUGGING_GUIDE.md for template and Dockerfile best practices.
