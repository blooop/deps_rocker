import unittest
import re
import pkg_resources
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class TestSnippetValidation(unittest.TestCase):
    """Tests to validate Dockerfile snippets for common template issues"""

    def test_dockerfile_snippets_have_no_unescaped_at_symbols(self):
        """
        Test that all Dockerfile snippets properly escape @ symbols with @@.

        This prevents empy template parsing errors when packages like @anthropic-ai/claude-code
        are used in npm install commands.
        """
        # Get all rocker extensions from this project
        project_extensions = []
        for entry_point in pkg_resources.iter_entry_points("rocker.extensions"):
            if entry_point.module_name.startswith("deps_rocker.extensions"):
                try:
                    ext_class = entry_point.load()
                    if issubclass(ext_class, SimpleRockerExtension):
                        project_extensions.append((entry_point.name, ext_class))
                except Exception:
                    # Skip extensions that can't be loaded
                    continue

        # Find extensions that have snippet files
        snippet_issues = []

        for ext_name, ext_class in project_extensions:
            # Try to find the snippet file
            ext_instance = ext_class()

            # Get the extension's directory
            module_path = Path(ext_class.__module__.replace(".", "/") + ".py")
            project_root = Path(__file__).parent.parent
            ext_dir = project_root / module_path.parent

            # Look for snippet files
            snippet_files = list(ext_dir.glob(f"{ext_name}_snippet.Dockerfile"))
            if not snippet_files:
                # Also check for any *_snippet.Dockerfile files
                snippet_files = list(ext_dir.glob("*_snippet.Dockerfile"))

            for snippet_file in snippet_files:
                if snippet_file.exists():
                    content = snippet_file.read_text()

                    # Look for unescaped @ symbols that could be problematic
                    # Pattern: @ followed by word characters but not preceded by @
                    # This catches things like "@anthropic-ai" but not "@@anthropic-ai"
                    unescaped_at_pattern = r"(?<!@)@[a-zA-Z][a-zA-Z0-9_/-]*"

                    matches = re.finditer(unescaped_at_pattern, content)
                    for match in matches:
                        # Skip common Dockerfile variables that should not be escaped
                        matched_text = match.group(0)

                        # Common variables/patterns that legitimately use @ and should not be escaped
                        skip_patterns = [
                            r"@\d+",  # version numbers like @16
                            r"@latest",  # version tags
                            r"@sha256:",  # docker image digests
                            r'@[a-zA-Z_][a-zA-Z0-9_]*\["',  # empy template dict access like @dep["path"]
                            r"@[a-zA-Z_][a-zA-Z0-9_]*$",  # simple empy template variables like @dep
                        ]

                        should_skip = any(
                            re.match(pattern, matched_text) for pattern in skip_patterns
                        )

                        # Also check if this is within empy template blocks
                        # Look for surrounding @[...] or @{...} patterns
                        surrounding_context = content[max(0, match.start() - 50) : match.end() + 50]
                        if "@[" in surrounding_context and "]@" in surrounding_context:
                            should_skip = True  # Likely within an empy template block

                        if should_skip:
                            continue

                        # Check if this looks like a package name (contains / or -)
                        if "/" in matched_text or "-" in matched_text:
                            # This looks like a package name that should be escaped
                            line_num = content[: match.start()].count("\n") + 1
                            snippet_issues.append(
                                f"{ext_name}: {snippet_file.name}:{line_num} - "
                                f"Unescaped '@' in '{matched_text}'. "
                                f"Should be '@@{matched_text[1:]}' to prevent empy template errors."
                            )

        # Report all issues
        if snippet_issues:
            error_msg = (
                "Found unescaped @ symbols in Dockerfile snippets that could cause empy template errors:\n"
                + "\n".join(snippet_issues)
                + "\n\nTo fix: Replace single @ with @@ in package names like npm packages (@scope/package -> @@scope/package)"
            )
            self.fail(error_msg)

    def test_snippet_files_can_be_loaded_without_template_errors(self):
        """
        Test that all snippet files can be processed by the template engine without errors.
        """
        # Get all rocker extensions from this project
        project_extensions = []
        for entry_point in pkg_resources.iter_entry_points("rocker.extensions"):
            if entry_point.module_name.startswith("deps_rocker.extensions"):
                try:
                    ext_class = entry_point.load()
                    if issubclass(ext_class, SimpleRockerExtension):
                        project_extensions.append((entry_point.name, ext_class))
                except Exception:
                    continue

        template_errors = []

        for ext_name, ext_class in project_extensions:
            try:
                ext_instance = ext_class()
                # Try to get the snippet - this will trigger template processing
                snippet = ext_instance.get_snippet({})

                # The snippet should be a string (empty string is fine)
                if snippet is not None and not isinstance(snippet, str):
                    template_errors.append(
                        f"{ext_name}: get_snippet() returned {type(snippet)}, expected str"
                    )

            except Exception as e:
                # Check if this is a template-related error
                error_str = str(e).lower()
                if any(
                    keyword in error_str
                    for keyword in ["template", "empy", "not defined", "name error"]
                ):
                    template_errors.append(f"{ext_name}: Template error - {e}")
                else:
                    # Other errors might be expected (missing dependencies, etc.)
                    # Just log them but don't fail the test
                    print(f"Note: {ext_name} had non-template error: {e}")

        if template_errors:
            error_msg = (
                "Found template processing errors in extensions:\n"
                + "\n".join(template_errors)
                + "\n\nThese are likely caused by unescaped @ symbols or other template syntax issues."
            )
            self.fail(error_msg)


if __name__ == "__main__":
    unittest.main()
