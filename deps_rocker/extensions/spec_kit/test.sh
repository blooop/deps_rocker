#!/bin/bash
set -e

echo "Testing Spec Kit installation..."

# Test if specify command is available
if command -v specify >/dev/null 2>&1; then
  echo "Spec Kit (specify) found in PATH"
  if specify --help >/dev/null 2>&1; then
    echo "Spec Kit is working"
  else
    echo "ERROR: Spec Kit found but not working properly" >&2
    exit 1
  fi
elif uv tool run specify-cli specify --help >/dev/null 2>&1; then
  echo "Spec Kit works via 'uv tool run specify-cli specify'"
else
  echo "ERROR: specify not accessible via PATH or uv tool run" >&2
  exit 1
fi

echo "spec_kit extension test completed successfully!"
