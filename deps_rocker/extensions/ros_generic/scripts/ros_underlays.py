#!/usr/bin/env python3
"""Helper CLI to build and manage ROS underlays from *.repos manifests."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from typing import Dict, List, Sequence

DEFAULT_UNDERLAY_ROOT = Path("/opt/ros/underlays")
DEFAULT_MANIFEST_FILE = DEFAULT_UNDERLAY_ROOT / "ros_underlays_manifest.json"
DEFAULT_SEARCH_ROOTS = ("/dependencies",)


def log(message: str, *, stream=sys.stdout) -> None:
    stream.write(f"[ros-underlays] {message}\n")
    stream.flush()


def verbose_log(enabled: bool, message: str) -> None:
    if enabled:
        log(message)


@dataclass
class ManifestEntry:
    """Represents a repos manifest tracked as an underlay."""

    identifier: str
    manifest_path: Path
    source_path: Path
    display_name: str
    order: int

    @classmethod
    def from_dict(cls, raw: dict) -> "ManifestEntry":
        manifest_path = Path(raw["manifest_path"]).expanduser().resolve()
        source_path = Path(raw["source_path"]).expanduser().resolve()
        display_name = raw.get("display_name", raw.get("relative_path", manifest_path.as_posix()))
        order = int(raw.get("order", 0))
        return cls(
            identifier=raw["id"],
            manifest_path=manifest_path,
            source_path=source_path,
            display_name=display_name,
            order=order,
        )

    def install_path(self, underlay_root: Path) -> Path:
        return underlay_root / self.identifier / "install"

    def build_path(self, underlay_root: Path) -> Path:
        return underlay_root / self.identifier / "build"

    def log_path(self, underlay_root: Path) -> Path:
        return underlay_root / self.identifier / "log"

    def stamp_path(self, underlay_root: Path) -> Path:
        return underlay_root / self.identifier / ".manifest.sha256"


def load_manifest_entries(
    manifest_file: Path,
    *,
    extra_roots: Sequence[str],
    force_rescan: bool,
    verbose: bool,
) -> tuple[List[ManifestEntry], List[str]]:
    manifest_data = {}
    search_roots: List[str] = list(DEFAULT_SEARCH_ROOTS)
    if manifest_file.is_file() and not force_rescan:
        try:
            manifest_data = json.loads(manifest_file.read_text(encoding="utf-8"))
            search_roots = manifest_data.get("search_roots", search_roots)
        except json.JSONDecodeError as exc:
            log(f"Warning: could not parse manifest file {manifest_file}: {exc}", stream=sys.stderr)
    for root in extra_roots:
        if root not in search_roots:
            search_roots.append(root)
    entries: List[ManifestEntry]
    raw_entries = manifest_data.get("manifests") if manifest_data and not force_rescan else None
    if raw_entries:
        entries = [ManifestEntry.from_dict(item) for item in raw_entries]
    else:
        entries = discover_manifests(search_roots, verbose=verbose)
        write_manifest_file(manifest_file, search_roots, entries)
    entries.sort(key=lambda item: (item.order, item.display_name))
    return entries, search_roots


def discover_manifests(search_roots: Sequence[str], *, verbose: bool) -> List[ManifestEntry]:
    entries: List[ManifestEntry] = []
    seen: set[str] = set()
    for root in search_roots:
        root_path = Path(root).expanduser()
        if not root_path.exists():
            verbose_log(verbose, f"Skipping missing search root {root_path}")
            continue
        for manifest in root_path.rglob("*.repos"):
            if not manifest.is_file():
                continue
            if _should_skip_path(manifest):
                verbose_log(verbose, f"Ignoring manifest under ignored path {manifest}")
                continue
            rel = manifest.relative_to(root_path)
            key = f"{root_path.resolve()}::{rel.as_posix()}"
            if key in seen:
                continue
            seen.add(key)
            identifier = _make_identifier(key)
            order = _order_hint(rel)
            entries.append(
                ManifestEntry(
                    identifier=identifier,
                    manifest_path=manifest.resolve(),
                    source_path=manifest.parent.resolve(),
                    display_name=rel.as_posix(),
                    order=order,
                )
            )
    return entries


def sort_manifest_entries(entries: Sequence[ManifestEntry]) -> List[ManifestEntry]:
    return sorted(entries, key=lambda item: (item.order, item.display_name))


def register_new_entries(
    entries_map: Dict[str, ManifestEntry], candidates: Sequence[ManifestEntry]
) -> List[ManifestEntry]:
    new_entries: List[ManifestEntry] = []
    for candidate in candidates:
        if candidate.identifier in entries_map:
            continue
        entries_map[candidate.identifier] = candidate
        new_entries.append(candidate)
    return new_entries


def write_manifest_file(
    manifest_file: Path, search_roots: Sequence[str], entries: Sequence[ManifestEntry]
) -> None:
    manifest_file.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "version": 1,
        "search_roots": list(search_roots),
        "manifests": [
            {
                "id": entry.identifier,
                "manifest_path": entry.manifest_path.as_posix(),
                "source_path": entry.source_path.as_posix(),
                "display_name": entry.display_name,
                "order": entry.order,
            }
            for entry in entries
        ],
    }
    manifest_file.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _should_skip_path(path: Path) -> bool:
    ignored = {"build", "install", "log", "logs", "__pycache__"}
    for part in path.parts:
        if part.startswith(".") and part not in {"."}:
            return True
        if part in ignored:
            return True
    return False


def _make_identifier(key: str) -> str:
    digest = sha256(key.encode("utf-8")).hexdigest()[:10]
    safe = "".join(ch if ch.isalnum() else "_" for ch in key)
    safe = "_".join(filter(None, safe.split("_")))
    safe = safe[:48] if safe else "underlay"
    return f"{safe}_{digest}"


def _order_hint(rel_path: Path) -> int:
    parts = rel_path.parts
    depth = len(parts)
    filename = rel_path.name.lower()
    depends_priority = 0 if "depends" in filename else 1
    return depends_priority * 100 + depth


def ensure_aggregator(underlay_root: Path) -> Path:
    underlay_root.mkdir(parents=True, exist_ok=True)
    aggregator = underlay_root / "setup.bash"
    if not aggregator.exists():
        aggregator.write_text(
            "#!/usr/bin/env bash\n"
            "# Auto-generated ROS underlay aggregator.\n"
            "# Currently no underlays have been built.\n",
            encoding="utf-8",
        )
        aggregator.chmod(0o755)
    return aggregator


def write_aggregator(install_paths: Sequence[Path], aggregator: Path) -> None:
    lines = [
        "#!/usr/bin/env bash",
        "# Auto-generated by ros-underlays. Do not edit manually.",
    ]
    exported = []
    for path in install_paths:
        setup = path / "setup.bash"
        if not setup.is_file():
            continue
        lines.append(f"if [ -f '{setup.as_posix()}' ]; then")
        lines.append(f"  source '{setup.as_posix()}'")
        lines.append("fi")
        exported.append(path.as_posix())
    lines.append(f"export ROS_UNDERLAY_PATHS='{':'.join(exported)}'")
    lines.append("")
    aggregator.write_text("\n".join(lines), encoding="utf-8")
    aggregator.chmod(0o755)


def compute_manifest_hash(manifest: Path) -> str:
    return sha256(manifest.read_bytes()).hexdigest()


def should_build(entry: ManifestEntry, *, underlay_root: Path, force: bool) -> bool:
    if force:
        return True
    stamp_file = entry.stamp_path(underlay_root)
    install_setup = entry.install_path(underlay_root) / "setup.bash"
    if not install_setup.is_file():
        return True
    if not stamp_file.is_file():
        return True
    current_hash = compute_manifest_hash(entry.manifest_path)
    stored_hash = stamp_file.read_text(encoding="utf-8").strip()
    return stored_hash != current_hash


def update_stamp(entry: ManifestEntry, *, underlay_root: Path) -> None:
    stamp_file = entry.stamp_path(underlay_root)
    stamp_file.parent.mkdir(parents=True, exist_ok=True)
    stamp_file.write_text(compute_manifest_hash(entry.manifest_path) + "\n", encoding="utf-8")


def run_vcs_import(entry: ManifestEntry, *, verbose: bool) -> None:
    if shutil.which("vcs") is None:
        log("vcstool (vcs) command not found; skipping import step", stream=sys.stderr)
        return
    entry.source_path.mkdir(parents=True, exist_ok=True)
    log(f"Importing repositories for {entry.display_name}")
    try:
        subprocess.run(
            ["vcs", "import", "--recursive", entry.source_path.as_posix()],
            input=entry.manifest_path.read_bytes(),
            check=False,
        )
    except FileNotFoundError:
        log("vcstool not available; skipping import", stream=sys.stderr)
    except subprocess.CalledProcessError as exc:
        log(f"vcs import reported an error (ignored): {exc}", stream=sys.stderr)
    verbose_log(verbose, f"Finished import for {entry.display_name}")


def has_ros_packages(
    entry: ManifestEntry, *, ros_setup: Path, aggregator: Path, verbose: bool
) -> bool:
    cmd = " ; ".join(
        [
            "set -euo pipefail",
            f"if [ -f {shlex.quote(ros_setup.as_posix())} ]; then source {shlex.quote(ros_setup.as_posix())}; fi",
            f"if [ -f {shlex.quote(aggregator.as_posix())} ]; then source {shlex.quote(aggregator.as_posix())}; fi",
            f"colcon list --names-only --base-paths {shlex.quote(entry.source_path.as_posix())}",
        ]
    )
    result = subprocess.run(
        ["bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        verbose_log(
            verbose, f"colcon list failed for {entry.display_name}: {result.stderr.strip()}"
        )
        return False
    return bool(result.stdout.strip())


def run_colcon_build(
    entry: ManifestEntry,
    *,
    ros_setup: Path,
    aggregator: Path,
    underlay_root: Path,
    colcon_args: Sequence[str],
    verbose: bool,
) -> None:
    install_dir = entry.install_path(underlay_root)
    build_dir = entry.build_path(underlay_root)
    log_dir = entry.log_path(underlay_root)
    install_dir.parent.mkdir(parents=True, exist_ok=True)
    install_dir.mkdir(parents=True, exist_ok=True)
    build_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    quoted_args = " ".join(shlex.quote(arg) for arg in colcon_args)
    cmd_parts = [
        "set -euo pipefail",
        f"source {shlex.quote(ros_setup.as_posix())}",
        f"if [ -f {shlex.quote(aggregator.as_posix())} ]; then source {shlex.quote(aggregator.as_posix())}; fi",
        f"colcon build --merge-install --base-paths {shlex.quote(entry.source_path.as_posix())} "
        f"--build-base {shlex.quote(build_dir.as_posix())} "
        f"--install-base {shlex.quote(install_dir.as_posix())} "
        f"--log-base {shlex.quote(log_dir.as_posix())} "
        "--event-handlers console_cohesion+",
    ]
    if quoted_args:
        cmd_parts[-1] = cmd_parts[-1] + " " + quoted_args
    command = " ; ".join(cmd_parts)
    log(f"Building underlay {entry.display_name}")
    subprocess.run(["bash", "-lc", command], check=True)
    verbose_log(verbose, f"Finished building underlay {entry.display_name}")


def clean_underlay(entry: ManifestEntry, *, underlay_root: Path, verbose: bool) -> None:
    for path in (
        entry.install_path(underlay_root),
        entry.build_path(underlay_root),
        entry.log_path(underlay_root),
        entry.stamp_path(underlay_root),
    ):
        if path.is_dir():
            shutil.rmtree(path, ignore_errors=True)
            verbose_log(verbose, f"Removed directory {path}")
        elif path.is_file():
            path.unlink(missing_ok=True)
            verbose_log(verbose, f"Removed file {path}")


def sync_underlays(args: argparse.Namespace) -> int:
    manifest_file = Path(args.manifest_file)
    underlay_root = Path(args.underlay_root)
    aggregator = ensure_aggregator(underlay_root)
    ros_distro = args.ros_distro or os.environ.get("ROS_DISTRO", "humble")
    ros_setup = Path(f"/opt/ros/{ros_distro}/setup.bash")
    entries, search_roots = load_manifest_entries(
        manifest_file,
        extra_roots=args.search_root,
        force_rescan=args.rescan,
        verbose=args.verbose,
    )
    entries_map: Dict[str, ManifestEntry] = {entry.identifier: entry for entry in entries}
    processed: set[str] = set()
    manifest_changed = False

    queue: List[ManifestEntry] = sort_manifest_entries(entries_map.values())
    if not queue:
        log("No *.repos manifests discovered; keeping empty aggregator")
        write_aggregator([], aggregator)
        return 0

    while queue:
        entry = queue.pop(0)
        if entry.identifier in processed:
            continue
        if args.action == "clean":
            clean_underlay(entry, underlay_root=underlay_root, verbose=args.verbose)
            processed.add(entry.identifier)
            continue
        if args.dry_run:
            log(f"[dry-run] Would process manifest {entry.display_name}")
            processed.add(entry.identifier)
            continue

        run_vcs_import(entry, verbose=args.verbose)

        discovered = discover_manifests([entry.source_path.as_posix()], verbose=args.verbose)
        new_entries = register_new_entries(entries_map, discovered)
        if new_entries:
            manifest_changed = True
            for new_entry in new_entries:
                verbose_log(
                    args.verbose,
                    f"Discovered dependent manifest {new_entry.display_name}",
                )
            queue = [
                item
                for item in sort_manifest_entries(entries_map.values())
                if item.identifier not in processed
            ]
            continue

        if not has_ros_packages(
            entry, ros_setup=ros_setup, aggregator=aggregator, verbose=args.verbose
        ):
            log(f"Skipping build for {entry.display_name}: no ROS packages detected")
            processed.add(entry.identifier)
            continue
        if should_build(entry, underlay_root=underlay_root, force=args.force):
            try:
                run_colcon_build(
                    entry,
                    ros_setup=ros_setup,
                    aggregator=aggregator,
                    underlay_root=underlay_root,
                    colcon_args=args.colcon_arg,
                    verbose=args.verbose,
                )
                update_stamp(entry, underlay_root=underlay_root)
            except subprocess.CalledProcessError as exc:
                log(f"colcon build failed for {entry.display_name}: {exc}", stream=sys.stderr)
                return exc.returncode or 1
        processed.add(entry.identifier)

    if args.action == "clean":
        write_aggregator([], aggregator)
    else:
        ordered_entries = sort_manifest_entries(entries_map.values())
        built_paths = [
            entry.install_path(underlay_root)
            for entry in ordered_entries
            if entry.install_path(underlay_root).is_dir()
        ]
        write_aggregator(built_paths, aggregator)

    if manifest_changed:
        write_manifest_file(
            manifest_file,
            search_roots,
            sort_manifest_entries(entries_map.values()),
        )
    return 0


def list_underlays(args: argparse.Namespace) -> int:
    manifest_file = Path(args.manifest_file)
    underlay_root = Path(args.underlay_root)
    entries, _ = load_manifest_entries(
        manifest_file, extra_roots=args.search_root, force_rescan=args.rescan, verbose=args.verbose
    )
    if not entries:
        log("No tracked underlays")
        return 0
    for entry in sort_manifest_entries(entries):
        install_dir = entry.install_path(underlay_root)
        status = "built" if install_dir.is_dir() else "missing"
        log(f"{status:7} :: {entry.display_name} -> {install_dir.as_posix()}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manage ROS underlays from *.repos manifests")
    parser.add_argument(
        "action",
        nargs="?",
        default="sync",
        choices=("sync", "list", "clean", "rebuild"),
        help="Operation to perform (default: sync)",
    )
    parser.add_argument(
        "--force", action="store_true", help="Force rebuild even if manifest unchanged"
    )
    parser.add_argument(
        "--rescan", action="store_true", help="Rescan search roots instead of cached manifest list"
    )
    parser.add_argument(
        "--manifest-file",
        default=DEFAULT_MANIFEST_FILE.as_posix(),
        help="Path to cached manifest metadata (default: /opt/ros/underlays/ros_underlays_manifest.json)",
    )
    parser.add_argument(
        "--underlay-root",
        default=DEFAULT_UNDERLAY_ROOT.as_posix(),
        help="Directory where underlays are stored (default: /opt/ros/underlays)",
    )
    parser.add_argument(
        "--search-root",
        action="append",
        default=[],
        help="Additional directories to scan for *.repos manifests",
    )
    parser.add_argument(
        "--colcon-arg", action="append", default=[], help="Additional arguments for colcon build"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Print actions without executing them"
    )
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    parser.add_argument("--ros-distro", help="Override ROS distribution to source for builds")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.action == "list":
        return list_underlays(args)
    if args.action == "rebuild":
        args.force = True
        args.rescan = True
        args.action = "sync"
    return sync_underlays(args)


if __name__ == "__main__":
    sys.exit(main())
