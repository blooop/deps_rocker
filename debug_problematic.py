#!/usr/bin/env python3

import em

# Test problematic line that might be causing empy issues
problematic_line = """printf "\\[ -s \\"\\$NVM_DIR/nvm.sh\\" \\] && . \\"\\$NVM_DIR/nvm.sh\\"\\n" >> "$OUTPUT_DIR/nvm-env.sh" &&"""

template_args = {
    "node_version": "24.9.0",
    "npm_version": "11.6.1",
    "nvm_version": "0.40.0",
    "base_image": "testfixture_extensions_base",
    "builder_stage": "npm_builder",
    "builder_output_dir": "/opt/deps_rocker/npm",
    "builder_output_path": "/opt/deps_rocker/npm/",
    "extension_name": "npm",
}

print("Testing problematic line:")
print(f"Original: {problematic_line}")

try:
    expanded = em.expand(problematic_line, template_args)
    print(f"Expanded: {expanded}")
except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()

# Test simpler version
simple_line = """printf "test" >> "$OUTPUT_DIR/nvm-env.sh" &&"""
print(f"\nTesting simple line: {simple_line}")
try:
    expanded = em.expand(simple_line, template_args)
    print(f"Expanded: {expanded}")
except Exception as e:
    print(f"Error: {e}")
