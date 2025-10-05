#!/usr/bin/env python3

import em
from deps_rocker.extensions.npm.npm import Npm

# Create an npm extension instance
npm_ext = Npm()

# Build template args like the extension would
template_args = npm_ext._build_template_args(
    cliargs={"base_image": "testfixture_extensions_base"}, empy_args=npm_ext.empy_builder_args
)

print("Template args:")
for k, v in template_args.items():
    print(f"  {k}: {repr(v)}")

# Try to process just the FROM line with new syntax
from_line_new = '@(f"FROM {base_image} AS {builder_stage}")\n'
print(f"\nNew FROM line: {from_line_new}")

try:
    expanded = em.expand(from_line_new, template_args)
    print(f"Expanded new FROM line: {repr(expanded)}")
except Exception as e:
    print(f"Error expanding new FROM line: {e}")

# Try a minimal complete template with new syntax
minimal_template_new = """# Test template  
@(f"FROM {base_image} AS {builder_stage}")
RUN echo "test"
"""

print("\nMinimal template with new syntax:")
try:
    expanded = em.expand(minimal_template_new, template_args)
    print(f"Expanded minimal template:\n{expanded}")
except Exception as e:
    print(f"Error expanding minimal template: {e}")
    import traceback

    traceback.print_exc()

# Try a minimal builder snippet
print("\nTesting minimal builder snippet:")
with open("debug_minimal_builder.Dockerfile", "r") as f:
    minimal_builder = f.read()

try:
    expanded = em.expand(minimal_builder, template_args)
    print(f"Minimal builder expanded successfully:\n{expanded}")
except Exception as e:
    print(f"Error expanding minimal builder: {e}")
    import traceback

    traceback.print_exc()

# Try the entire builder snippet
try:
    snippet = npm_ext.get_builder_snippet(cliargs={"base_image": "testfixture_extensions_base"})
    print(f"\nBuilder snippet generated successfully, length: {len(snippet)}")
    print("First few lines:")
    print("\n".join(snippet.split("\n")[:10]))
except Exception as e:
    print(f"\nError generating builder snippet: {e}")
    import traceback

    traceback.print_exc()
