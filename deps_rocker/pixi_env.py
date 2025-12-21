from __future__ import annotations

from dataclasses import dataclass, field
import logging
from typing import Dict, Iterable, List, Optional, Sequence

# pylint: disable=too-many-positional-arguments

import toml

DEFAULT_CHANNELS: tuple[str, ...] = ("conda-forge",)
DEFAULT_PLATFORMS: tuple[str, ...] = ("linux-64",)
DEFAULT_ENV_NAME = "tools"


def _dedupe_preserve_order(items: Iterable[str]) -> list[str]:
    seen = set()
    ordered: list[str] = []
    for item in items:
        if item not in seen and item is not None:
            seen.add(item)
            ordered.append(item)
    return ordered


@dataclass
class PixiEnvData:
    name: str
    dependencies: Dict[str, str] = field(default_factory=dict)
    pypi_dependencies: Dict[str, str] = field(default_factory=dict)
    channels: List[str] = field(default_factory=list)
    platforms: List[str] = field(default_factory=list)


class PixiEnvironmentCollector:
    """Collect pixi dependency contributions from active extensions for a single run."""

    def __init__(self) -> None:
        self._run_counter = 0
        self.reset_for_new_run()

    def reset_for_new_run(self) -> None:
        """Clear all collected state for a new rocker invocation."""

        self._run_counter += 1
        self.envs: Dict[str, PixiEnvData] = {}
        self._sources: Dict[tuple[str, str, str], str] = {}

    def register_from_extension(self, extension) -> None:
        """Record pixi contributions declared on a SimpleRockerExtension."""

        env_name: Optional[str] = getattr(extension, "pixi_environment", DEFAULT_ENV_NAME)
        dependencies: Dict[str, str] = getattr(extension, "pixi_dependencies", {}) or {}
        pypi_dependencies: Dict[str, str] = getattr(extension, "pixi_pypi_dependencies", {}) or {}
        channels: Sequence[str] = getattr(extension, "pixi_channels", ()) or ()
        platforms: Sequence[str] = getattr(extension, "pixi_platforms", ()) or ()

        self.register_contribution(
            extension_name=getattr(extension, "name", extension.__class__.__name__),
            env_name=env_name,
            dependencies=dependencies,
            pypi_dependencies=pypi_dependencies,
            channels=list(channels),
            platforms=list(platforms),
        )

    def register_contribution(
        self,
        *,
        extension_name: str,
        env_name: Optional[str],
        dependencies: Dict[str, str] | None = None,
        pypi_dependencies: Dict[str, str] | None = None,
        channels: Sequence[str] | None = None,
        platforms: Sequence[str] | None = None,
    ) -> None:  # pylint: disable=too-many-positional-arguments,too-many-arguments
        """Register a contribution to the pixi environment."""

        if not env_name:
            return

        deps = dependencies or {}
        pypi_deps = pypi_dependencies or {}
        ch_list = list(channels or [])
        platform_list = list(platforms or [])
        if not deps and not pypi_deps and not ch_list and not platform_list:
            return

        spec = self.envs.setdefault(env_name, PixiEnvData(name=env_name))
        self._merge_dependencies(spec.dependencies, deps, env_name, extension_name, "dependencies")
        self._merge_dependencies(
            spec.pypi_dependencies, pypi_deps, env_name, extension_name, "pypi-dependencies"
        )
        spec.channels = _dedupe_preserve_order([*spec.channels, *ch_list])
        spec.platforms = _dedupe_preserve_order([*spec.platforms, *platform_list])

    def _merge_dependencies(
        self,
        target: Dict[str, str],
        incoming: Dict[str, str],
        env_name: str,
        extension_name: str,
        section: str,
    ) -> None:
        for pkg, constraint in incoming.items():
            if pkg in target and target[pkg] != constraint:
                logging.warning(
                    "Pixi dependency conflict for env '%s': package '%s' already set to '%s',"
                    " ignoring '%s' from extension '%s'",
                    env_name,
                    pkg,
                    target[pkg],
                    constraint,
                    extension_name,
                )
                continue
            target.setdefault(pkg, constraint)
            self._sources[(env_name, section, pkg)] = extension_name

    def ensure_env(self, env_name: str) -> PixiEnvData:
        return self.envs.setdefault(env_name, PixiEnvData(name=env_name))

    def merged_channels(self, default_channels: Sequence[str] = DEFAULT_CHANNELS) -> list[str]:
        collected = list(default_channels)
        for env in self.envs.values():
            collected.extend(env.channels)
        return _dedupe_preserve_order(collected)

    def merged_platforms(self, default_platforms: Sequence[str] = DEFAULT_PLATFORMS) -> list[str]:
        collected = list(default_platforms)
        for env in self.envs.values():
            collected.extend(env.platforms)
        return _dedupe_preserve_order(collected)

    def render_manifest(
        self,
        *,
        env_name: str = DEFAULT_ENV_NAME,
        project_name: str = "deps-rocker-tools",
        default_channels: Sequence[str] = DEFAULT_CHANNELS,
        default_platforms: Sequence[str] = DEFAULT_PLATFORMS,
    ) -> str:
        """Render a pixi.toml manifest covering all collected environments."""

        self.ensure_env(env_name)

        manifest: dict = {
            "project": {
                "name": project_name,
                "version": "0.0.0",
                "channels": self.merged_channels(default_channels),
                "platforms": self.merged_platforms(default_platforms),
            },
            "environments": {},
        }

        features: dict[str, dict] = {}
        for env, spec in sorted(self.envs.items(), key=lambda item: item[0]):
            feature_entry: dict[str, dict] = {}
            if spec.dependencies:
                feature_entry["dependencies"] = {
                    pkg: spec.dependencies[pkg] for pkg in sorted(spec.dependencies)
                }
            if spec.pypi_dependencies:
                feature_entry["pypi-dependencies"] = {
                    pkg: spec.pypi_dependencies[pkg] for pkg in sorted(spec.pypi_dependencies)
                }
            features[env] = feature_entry
            manifest["environments"][env] = {"features": [env], "solve-group": env}

        manifest["feature"] = features

        return toml.dumps(manifest)


_COLLECTOR = PixiEnvironmentCollector()


def get_pixi_collector() -> PixiEnvironmentCollector:
    return _COLLECTOR
