"""Patching utilities that integrate deps_rocker with rocker."""

from __future__ import annotations

import json
from json import JSONDecodeError
from typing import Any, Callable, Dict, Optional

from deps_rocker.buildkit import is_buildkit_enabled
from deps_rocker.docker_cli import docker_build_with_cli
from deps_rocker.docker_sdk import docker_build_with_sdk

OutputCallback = Optional[Callable[[str], None]]


def _format_output(message: Optional[str], callback: OutputCallback) -> None:
    if message and callback:
        callback(message)


def _normalise_buildargs(
    buildargs: Any,
    output_callback: OutputCallback,
) -> Optional[Dict[str, Any]]:
    if buildargs is None or isinstance(buildargs, dict):
        return buildargs

    if isinstance(buildargs, str):
        try:
            return json.loads(buildargs)
        except JSONDecodeError as exc:
            _format_output(
                f"Unable to decode build args string as JSON: {exc}; ignoring", output_callback
            )
            return None

    return None


def patch_rocker_docker_build() -> None:
    # Defer import so rocker remains optional at installation time
    try:
        from rocker import core
    except Exception:
        return

    if getattr(core, "_deps_rocker_buildkit_patch", False):
        return

    def patched_docker_build(
        docker_client=None,
        output_callback: OutputCallback = None,
        **kwargs: Any,
    ) -> Optional[str]:
        if is_buildkit_enabled():
            path = kwargs.get("path")
            if path:
                buildargs_for_cli = _normalise_buildargs(kwargs.get("buildargs"), output_callback)
                dockerfile = kwargs.get("dockerfile")
                try:
                    image_id = docker_build_with_cli(
                        path=path,
                        tag=kwargs.get("tag"),
                        rm=kwargs.get("rm", True),
                        nocache=kwargs.get("nocache", False),
                        pull=kwargs.get("pull", False),
                        dockerfile=dockerfile,
                        buildargs=buildargs_for_cli,
                        output_callback=output_callback,
                    )
                except FileNotFoundError:
                    _format_output(
                        "docker CLI not available; falling back to docker SDK build",
                        output_callback,
                    )
                    image_id = None

                if image_id:
                    return image_id

                _format_output(
                    "docker CLI build failed; attempting docker SDK build",
                    output_callback,
                )

        return docker_build_with_sdk(
            docker_client,
            output_callback,
            **kwargs,
        )

    core.docker_build = patched_docker_build
    core._deps_rocker_buildkit_patch = True  # pylint: disable=protected-access


__all__ = ["patch_rocker_docker_build"]
