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

    def _detect_files_in_workspace(self, _cliargs: dict) -> set[str]:
        """
        Detect files in the workspace and return a set of extension names to enable, in parallel.
        """
        import yaml

        workspace = self._resolve_workspace(_cliargs)

        extensions_dir = Path(__file__).parent.parent
        file_patterns = {}
        content_search_patterns = {}
        dir_patterns = {}
        for ext_dir in extensions_dir.iterdir():
            if not ext_dir.is_dir():
                continue
            rule_file = ext_dir / "auto_detect.yml"
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
                # Directory patterns
                for dname in rules.get("config_dirs", []):
                    dir_patterns[dname] = ext_name

        # Prepare detection functions and their arguments
        # Use only patterns from auto_detect.yml files for all extensions
        tasks = [
            (self._detect_exact_dir, (workspace, dir_patterns)),
            (self._detect_glob_patterns, (workspace, file_patterns)),
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
                            if search == "[tool.pixi]":
                                found_section = "[tool.pixi]" in content
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

    def _detect_glob_patterns(self, workspace, file_patterns):
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
        content_search_patterns = getattr(self, "_content_search_patterns", {})
        for pattern, ext in file_patterns.items():
            start = time.time()
            matches = [f for f in all_files if fnmatch.fnmatch(f, pattern)]
            duration = time.time() - start
            content_search_required = pattern in content_search_patterns
            # Special handling for pyproject.toml: uv should only activate if [tool.pixi] is NOT present
            if pattern == "pyproject.toml":
                for f in matches:
                    fpath = os.path.join(workspace_path, f)
                    try:
                        with open(fpath, "r", encoding="utf-8") as file:
                            content = file.read()
                        pixi_section = False
                        if "[tool.pixi]" in content:
                            pixi_section = True
                        if ext == "pixi":
                            # pixi handled by content_search, skip here
                            continue
                        if ext == "uv":
                            if not pixi_section:
                                print(
                                    f"{GREEN}[auto-detect] {ext}: ✓ Detected {pattern} ({f}) without [tool.pixi] -> enabling [search took {duration:.3f}s]{RESET}"
                                )
                                found.add(ext)
                            else:
                                print(
                                    f"{CYAN}[auto-detect] {ext}: Detected {pattern} ({f}) but [tool.pixi] present, NOT enabling [search took {duration:.3f}s]{RESET}"
                                )
                    except Exception as e:
                        print(f"{CYAN}[auto-detect] {ext}: Error reading {fpath}: {e}{RESET}")
                continue
            if matches:
                if content_search_required:
                    print(
                        f"{CYAN}[auto-detect] {ext}: Detected {pattern} ({len(matches)} matches), but content search required. Will check contents next. [search took {duration:.3f}s]{RESET}"
                    )
                else:
                    print(
                        f"{GREEN}[auto-detect] {ext}: ✓ Detected {pattern} ({len(matches)} matches) -> enabling [search took {duration:.3f}s]{RESET}"
                    )
                    found.add(ext)
            else:
                print(
                    f"{CYAN}[auto-detect] {ext}: Pattern {pattern} found no matches [search took {duration:.3f}s]{RESET}"
                )
        print(f"[auto-detect] Total file walk and match time: {time.time() - start_total:.3f}s")
        return found

    def _detect_exact_dir(self, workspace, patterns):
        found = set()
        # Check in workspace
        for dname, ext in patterns.items():
            dir_path = workspace / dname
            if dir_path.is_dir():
                print(
                    f"\033[92m[auto-detect] {ext}: ✓ Detected {dname} directory in workspace -> enabling\033[0m"
                )
                found.add(ext)
        # Check in user's home directory
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

        Args:
            cliargs: CLI arguments dict

        Returns:
            Set of extension names to enable
        """
        return self._detect_files_in_workspace(cliargs)
