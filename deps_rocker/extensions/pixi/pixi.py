from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path
from typing import Optional

from deps_rocker.buildkit import is_buildkit_enabled
from deps_rocker.pixi_env import (
    DEFAULT_CHANNELS,
    DEFAULT_ENV_NAME,
    DEFAULT_PLATFORMS,
    get_pixi_collector,
)
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


@dataclass
class PixiConfig:
    builder_image: str
    builder_tag: str
    env_name: str
    runtime_root: str
    cache_mounts: bool
    lock_in_context: bool
    manifest_name: str = "pixi.toml"
    lock_name: str = "pixi.lock"
    channels: list[str] | None = None
    platforms: list[str] | None = None

    @property
    def builder_image_ref(self) -> str:
        return f"{self.builder_image}:{self.builder_tag}"

    @property
    def env_bin_path(self) -> str:
        return f"{self.runtime_root}/.pixi/envs/{self.env_name}/bin"

    @property
    def runtime_home(self) -> str:
        return f"{self.runtime_root}/.pixi"


class Pixi(SimpleRockerExtension):
    """Install pixi and bake a build-time tools environment"""

    name = "pixi"
    depends_on_extension: tuple[str, ...] = ("user",)
    pixi_environment: Optional[str] = DEFAULT_ENV_NAME

    DEFAULT_BUILDER_IMAGE = "ghcr.io/prefix-dev/pixi"
    DEFAULT_BUILDER_TAG = "latest"
    DEFAULT_RUNTIME_ROOT = "/opt/pixi"

    def __init__(self):
        super().__init__()
        self._config: Optional[PixiConfig] = None
        self._manifest: Optional[str] = None
        self._lock_bytes: Optional[bytes] = None
        self._lock_path: Optional[Path] = None

    @staticmethod
    def register_arguments(parser, defaults=None):
        defaults = defaults or {}
        parser.add_argument(
            "--pixi-builder-image",
            type=str,
            default=defaults.get("pixi_builder_image", Pixi.DEFAULT_BUILDER_IMAGE),
            help="Builder image to use for pixi installs (defaults to official pixi image).",
        )
        parser.add_argument(
            "--pixi-builder-tag",
            type=str,
            default=defaults.get("pixi_builder_tag", Pixi.DEFAULT_BUILDER_TAG),
            help="Tag for pixi builder image (defaults to a stable pixi image; override with a jammy/matching base tag if needed).",
        )
        parser.add_argument(
            "--pixi-env-name",
            type=str,
            default=defaults.get("pixi_env_name", DEFAULT_ENV_NAME),
            help="Pixi environment name to build into the image (default: tools).",
        )
        parser.add_argument(
            "--pixi-runtime-root",
            type=str,
            default=defaults.get("pixi_runtime_root", Pixi.DEFAULT_RUNTIME_ROOT),
            help="Runtime root where the baked pixi environment will live (default: /opt/pixi).",
        )
        parser.add_argument(
            "--pixi-channel",
            action="append",
            dest="pixi_extra_channels",
            default=defaults.get("pixi_extra_channels"),
            help="Additional pixi channels to include (can be passed multiple times).",
        )
        parser.add_argument(
            "--pixi-platform",
            action="append",
            dest="pixi_extra_platforms",
            default=defaults.get("pixi_extra_platforms"),
            help="Additional pixi target platforms (defaults to linux-64).",
        )
        parser.add_argument(
            "--pixi-lockfile",
            type=str,
            default=defaults.get("pixi_lockfile"),
            help="Optional path to a pixi lockfile to copy into the build context for locked installs.",
        )

    def _resolve_config(self, cliargs) -> PixiConfig:
        if self._config:
            return self._config

        collector = get_pixi_collector()

        builder_image = cliargs.get("pixi_builder_image") or self.DEFAULT_BUILDER_IMAGE
        builder_tag = cliargs.get("pixi_builder_tag") or self.DEFAULT_BUILDER_TAG
        env_name = cliargs.get("pixi_env_name") or DEFAULT_ENV_NAME
        runtime_root = cliargs.get("pixi_runtime_root") or self.DEFAULT_RUNTIME_ROOT

        extra_channels = cliargs.get("pixi_extra_channels") or []
        extra_platforms = cliargs.get("pixi_extra_platforms") or []
        channels = collector.merged_channels([*DEFAULT_CHANNELS, *extra_channels])
        platforms = collector.merged_platforms([*DEFAULT_PLATFORMS, *extra_platforms])

        self._manifest = collector.render_manifest(
            env_name=env_name, default_channels=channels, default_platforms=platforms
        )

        lockfile_path = cliargs.get("pixi_lockfile")
        lock_in_context = False
        if lockfile_path:
            resolved_lock = Path(lockfile_path).expanduser()
            if resolved_lock.is_file():
                self._lock_bytes = resolved_lock.read_bytes()
                self._lock_path = resolved_lock
                lock_in_context = True
            else:
                logging.warning(
                    "Pixi lockfile not found at %s, continuing without it.", resolved_lock
                )

        cache_mounts = is_buildkit_enabled()

        self._config = PixiConfig(
            builder_image=builder_image,
            builder_tag=builder_tag,
            env_name=env_name,
            runtime_root=runtime_root,
            cache_mounts=cache_mounts,
            lock_in_context=lock_in_context,
            channels=channels,
            platforms=platforms,
        )

        shared_args = {
            "pixi_env_name": env_name,
            "pixi_runtime_root": runtime_root,
            "pixi_env_bin_path": self._config.env_bin_path,
            "pixi_manifest_name": self._config.manifest_name,
            "pixi_lock_name": self._config.lock_name,
            "pixi_lock_present": lock_in_context,
        }
        self.empy_args = shared_args
        self.empy_builder_args = {
            **shared_args,
            "pixi_builder_image": builder_image,
            "pixi_builder_tag": builder_tag,
            "pixi_builder_image_ref": self._config.builder_image_ref,
            "pixi_use_cache_mounts": cache_mounts,
        }
        return self._config

    def _ensure_config(self, cliargs) -> PixiConfig:
        return self._resolve_config(cliargs or {})

    def get_preamble(self, cliargs):
        self._ensure_config(cliargs)
        return super().get_preamble(cliargs)

    def get_snippet(self, cliargs=None):
        self._ensure_config(cliargs)
        return super().get_snippet(cliargs)

    def get_user_snippet(self, cliargs):
        self._ensure_config(cliargs)
        return super().get_user_snippet(cliargs)

    def get_files(self, cliargs):
        config = self._ensure_config(cliargs)
        files = {config.manifest_name: self._manifest or ""}
        if self._lock_bytes is not None:
            files[config.lock_name] = self._lock_bytes
        return files
