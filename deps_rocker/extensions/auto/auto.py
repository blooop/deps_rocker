"""Automatically detect and enable extensions based on workspace files"""

from rocker.extensions import RockerExtension
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed  # pylint: disable=E0611


class Auto(RockerExtension):
    def __init__(self):
        super().__init__()
        self._content_search_patterns = {}

    def _resolve_workspace(self, cliargs):
        root = cliargs.get("auto")
        # If root is True (from --auto with no value), treat as None
        if root is True or root is None or not isinstance(root, (str, Path)):
            path = Path.cwd().expanduser().resolve()
        else:
            path = Path(root).expanduser().resolve()
        print(f"[auto-detect] Scanning workspace: {path}")
        print(f"[auto-detect] Workspace exists: {path.exists()}")
        print(f"[auto-detect] Workspace is_dir: {path.is_dir()}")
        return path

    # Detect project files and enable relevant extensions based on workspace contents.
    # Use --auto=~/renv to specify the root directory for recursive search.

    @classmethod
    def get_name(cls):
        return cls.name

    @staticmethod
    def register_arguments(parser, defaults=None):
        """
        Register command-line arguments for the auto extension.
        """
        parser.add_argument(
            "--auto",
            type=str,
            nargs="?",
            const=str(Path.cwd()),
            help="Enable auto extension and optionally specify a search root directory. Defaults to current working directory.",
        )

    name = "auto"

    def _detect_files_in_workspace(self, _cliargs: dict, check_home: bool = True) -> set[str]:
        """
        Detect files in the workspace and return a set of extension names to enable, in parallel.

        Args:
            _cliargs: CLI arguments dict
            check_home: Whether to check home directory for config directories (default: True)
        """
        import yaml

        workspace = self._resolve_workspace(_cliargs)

        extensions_dir = Path(__file__).parent.parent
        file_patterns = {}
        content_search_patterns = {}
        exclude_content_patterns = {}
        dir_patterns = {}
        for ext_dir in extensions_dir.iterdir():
            if not ext_dir.is_dir():
                continue
            rule_file = ext_dir / "auto_detect.yaml"
            if rule_file.exists():
                with rule_file.open() as f:
                    rules = yaml.safe_load(f)
                ext_name = ext_dir.name
                # File patterns and content search
                for entry in rules.get("files", []):
                    if isinstance(entry, str):
                        file_patterns[entry] = ext_name
                    elif isinstance(entry, dict):
                        for fname, opts in entry.items():
                            file_patterns[fname] = ext_name
                            if "content_search" in opts:
                                content_search_patterns[fname] = {
                                    "ext": ext_name,
                                    "search": opts["content_search"],
                                }
                            if "exclude_content" in opts:
                                exclude_content_patterns[fname] = {
                                    "ext": ext_name,
                                    "exclude": opts["exclude_content"],
                                }
                # Directory patterns
                for dname in rules.get("config_dirs", []):
                    dir_patterns[dname] = ext_name

        # Prepare detection functions and their arguments
        # Use only patterns from auto_detect.yml files for all extensions
        tasks = [
            (self._detect_exact_dir, (workspace, dir_patterns, check_home)),
            (
                self._detect_glob_patterns,
                (workspace, file_patterns, exclude_content_patterns, content_search_patterns),
            ),
            (self._detect_content_search, (workspace, content_search_patterns)),
        ]

        results = set()
        print("[auto-detect] Running detection tasks in parallel...")
        with ThreadPoolExecutor() as executor:
            future_to_task = {executor.submit(func, *args): func.__name__ for func, args in tasks}
            for future in as_completed(future_to_task):
                task_name = future_to_task[future]
                try:
                    res = future.result()
                    # Color and task summary output
                    CYAN = "\033[96m"
                    GREEN = "\033[92m"
                    RESET = "\033[0m"
                    color = GREEN if res else CYAN
                    print(f"{color}[auto-detect] {task_name} detected: {res}{RESET}")
                    results |= res
                except Exception as e:
                    print(f"    \033[91m[auto-detect] {task_name} failed: {e}\033[0m")

        print(f"[auto-detect] Final detected extensions: {results}")
        return results

    def _detect_content_search(self, workspace, content_search_patterns):
        found = set()
        import os
        import re

        GREEN = "\033[92m"
        CYAN = "\033[96m"
        RESET = "\033[0m"
        self._content_search_patterns = content_search_patterns
        for fname, opts in content_search_patterns.items():
            ext = opts["ext"]
            search = opts["search"]
            found_file = False
            for root, _, files in os.walk(str(workspace), followlinks=False):
                for file in files:
                    if file == fname:
                        found_file = True
                        fpath = os.path.join(root, file)
                        try:
                            with open(fpath, "r", encoding="utf-8") as f:
                                content = f.read()
                            # Use plain substring match for '[tool.pixi]' pattern
                            # Special case for pixi to handle multiple content search patterns
                            if search in ["[tool.pixi.project]", "[tool.pixi]"]:
                                # Match either [tool.pixi] or [tool.pixi.project]
                                found_section = bool(
                                    re.search(
                                        r"^\s*\[tool\.pixi(\.project)?\]\s*$", content, re.MULTILINE
                                    )
                                )
                            else:
                                found_section = bool(re.search(search, content, re.MULTILINE))
                            if found_section:
                                print(
                                    f"{GREEN}[auto-detect] {ext}: Found content '{search}' in {fpath} -> enabling{RESET}"
                                )
                                found.add(ext)
                            else:
                                print(
                                    f"{CYAN}[auto-detect] {ext}: Found {fname} but content '{search}' NOT found in {fpath} -> NOT enabling{RESET}"
                                )
                        except Exception as e:
                            print(f"{CYAN}[auto-detect] {ext}: Error reading {fpath}: {e}{RESET}")
            if not found_file:
                print(
                    f"{CYAN}[auto-detect] {ext}: Content search: {fname} not found in workspace.{RESET}"
                )
        return found

    def _detect_glob_patterns(
        self, workspace, file_patterns, exclude_content_patterns, content_search_patterns
    ):
        import time
        import os
        import fnmatch

        GREEN = "\033[92m"
        CYAN = "\033[96m"
        RESET = "\033[0m"

        found = set()
        start_total = time.time()
        all_files = []
        workspace_path = str(workspace)
        walk_errors = []

        def walk_onerror(err):
            walk_errors.append(err)

        for root, dirs, files in os.walk(workspace_path, onerror=walk_onerror, followlinks=False):
            dirs[:] = [d for d in dirs if not os.path.islink(os.path.join(root, d))]
            for fname in files:
                fpath = os.path.join(root, fname)
                if os.path.islink(fpath):
                    continue
                try:
                    relpath = os.path.relpath(fpath, workspace_path)
                    all_files.append(relpath)
                except Exception as e:
                    walk_errors.append(e)
        # Match patterns in memory
        for pattern, ext in file_patterns.items():
            start = time.time()
            # Support both basename patterns (e.g., "package.xml") and full path patterns (e.g., ".cargo/config.toml")
            # Use pathlib for robust cross-platform path handling
            # Path is already imported at the top of the file
            pattern_path = Path(pattern)
            # If pattern_path has more than one part, treat as full path pattern
            if len(pattern_path.parts) > 1:
                # Full path pattern - use early termination for performance
                matches = []
                match_count = 0
                for f in all_files:
                    if Path(f).match(pattern):
                        matches.append(str(f))
                        match_count += 1
                        # For performance, we only need to know if matches exist for most patterns
                        # Only collect all matches for patterns that need content search or exclude_content
                        if (
                            pattern not in content_search_patterns
                            and pattern not in exclude_content_patterns
                            and match_count >= 1
                        ):
                            break
            else:
                # Basename pattern - match against filename only with early termination
                matches = []
                match_count = 0
                for f in all_files:
                    if fnmatch.fnmatch(Path(f).name, pattern):
                        matches.append(str(f))
                        match_count += 1
                        # For performance, we only need to know if matches exist for most patterns
                        if (
                            pattern not in content_search_patterns
                            and pattern not in exclude_content_patterns
                            and match_count >= 1
                        ):
                            break
            duration = time.time() - start
            content_search_required = pattern in content_search_patterns
            exclude_content_required = (
                pattern in exclude_content_patterns
                and exclude_content_patterns[pattern]["ext"] == ext
            )

            # Handle exclude_content patterns generically
            if exclude_content_required:
                exclude_pattern = exclude_content_patterns[pattern]["exclude"]
                for f in matches:
                    fpath = os.path.join(workspace_path, f)
                    try:
                        with open(fpath, "r", encoding="utf-8") as file:
                            content = file.read()
                        if exclude_pattern not in content:
                            print(
                                f"{GREEN}[auto-detect] {ext}: ✓ Detected {pattern} without '{exclude_pattern}' -> enabling [search took {duration:.3f}s]{RESET}"
                            )
                            found.add(ext)
                        else:
                            print(
                                f"{CYAN}[auto-detect] {ext}: Detected {pattern} but '{exclude_pattern}' present, NOT enabling [search took {duration:.3f}s]{RESET}"
                            )
                    except Exception as e:
                        print(f"{CYAN}[auto-detect] {ext}: Error reading {fpath}: {e}{RESET}")
                continue
            if matches:
                if content_search_required:
                    print(
                        f"{CYAN}[auto-detect] {ext}: Detected {pattern}, but content search required. Will check contents next. [search took {duration:.3f}s]{RESET}"
                    )
                else:
                    print(
                        f"{GREEN}[auto-detect] {ext}: ✓ Detected {pattern} -> enabling [search took {duration:.3f}s]{RESET}"
                    )
                    found.add(ext)
            else:
                print(
                    f"{CYAN}[auto-detect] {ext}: Pattern {pattern} found no matches [search took {duration:.3f}s]{RESET}"
                )
        print(f"[auto-detect] Total file walk and match time: {time.time() - start_total:.3f}s")
        return found

    def _detect_exact_dir(self, workspace, patterns, check_home=True):
        found = set()
        # Check in workspace
        for dname, ext in patterns.items():
            dir_path = workspace / dname
            if dir_path.is_dir():
                print(
                    f"\033[92m[auto-detect] {ext}: ✓ Detected {dname} directory in workspace -> enabling\033[0m"
                )
                found.add(ext)
        # Check in user's home directory (can be disabled for testing)
        if check_home:
            home = Path.home()
            for dname, ext in patterns.items():
                dir_path = home / dname
                if dir_path.is_dir():
                    print(
                        f"\033[92m[auto-detect] {ext}: ✓ Detected {dname} directory in home -> enabling\033[0m"
                    )
                    found.add(ext)
        return found

    def required(self, cliargs: dict) -> set[str]:
        """
        Returns a set of dependencies required by this extension based on detected files.

        This method returns the directly detected extensions AND their transitive dependencies
        to ensure proper ordering. Rocker's topological sort will handle the rest.

        Args:
            cliargs: CLI arguments dict

        Returns:
            Set of extension names to enable (detected + their transitive dependencies)
        """
        from rocker.core import list_plugins

        detected_extensions = self._detect_files_in_workspace(cliargs)

        # Collect transitive dependencies of detected extensions with cycle detection
        # This is necessary because when auto is used standalone (not through RockerExtensionManager),
        # the dependencies need to be explicitly returned for proper ordering.
        all_plugins = list_plugins()
        all_required = set()
        visited = set()  # Track visited extensions to prevent infinite recursion

        def collect_deps(ext_name: str):
            """Recursively collect dependencies with cycle detection"""
            if ext_name in visited:
                return  # Already processed, avoid cycle
            visited.add(ext_name)
            all_required.add(ext_name)

            if ext_name in all_plugins:
                ext_class = all_plugins[ext_name]
                ext_instance = ext_class()
                # Recursively collect dependencies
                ext_deps = ext_instance.required(cliargs)
                for dep in ext_deps:
                    collect_deps(dep)

        # Start with detected extensions
        for ext_name in detected_extensions:
            collect_deps(ext_name)

        return all_required
