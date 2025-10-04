"""deps_rocker package initialisation and rocker integration helpers."""

from __future__ import annotations

import os
import subprocess
import tempfile
from typing import Any, Callable, Dict, Optional


def _buildkit_requested() -> bool:
    value = os.environ.get("DOCKER_BUILDKIT", "")
    return value.strip().lower() in {"1", "true", "yes"}


def _format_output(message: str, callback: Optional[Callable[[str], None]]) -> None:
    if message and callback:
        callback(message)


def _docker_build_with_cli(
    *,
    path: str,
    tag: Optional[str],
    rm: bool,
    nocache: bool,
    pull: bool,
    dockerfile: Optional[str],
    buildargs: Optional[Dict[str, Any]],
    output_callback: Optional[Callable[[str], None]],
) -> Optional[str]:
    env = os.environ.copy()
    env.setdefault("DOCKER_BUILDKIT", "1")

    with tempfile.NamedTemporaryFile(delete=False) as iid_file:
        iid_path = iid_file.name

    cmd = [
        "docker",
        "build",
        "--progress=plain",
        f"--iidfile={iid_path}",
    ]

    if nocache:
        cmd.append("--no-cache")
    if pull:
        cmd.append("--pull")
    if not rm:
        cmd.append("--rm=false")
    if dockerfile:
        cmd.extend(["-f", dockerfile])
    if buildargs:
        for key, value in buildargs.items():
            cmd.extend(["--build-arg", f"{key}={value}"])
    if tag:
        cmd.extend(["-t", tag])
    cmd.append(path)

    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
    except FileNotFoundError:
        os.unlink(iid_path)
        raise

    try:
        assert process.stdout is not None
        for raw_line in process.stdout:
            line = raw_line.rstrip("\n")
            _format_output(line, output_callback)
        process.wait()
    finally:
        if process.stdout is not None:
            process.stdout.close()

    if process.returncode != 0:
        os.unlink(iid_path)
        return None

    try:
        with open(iid_path, "r", encoding="utf-8") as fh:
            image_identifier = fh.read().strip()
    finally:
        os.unlink(iid_path)

    if not image_identifier:
        return None

    if image_identifier.startswith("sha256:"):
        image_identifier = image_identifier.split(":", 1)[1]

    return image_identifier[:12]


def _docker_build_with_sdk(
    docker_client,
    output_callback: Optional[Callable[[str], None]],
    **kwargs: Any,
) -> Optional[str]:
    import re

    if not docker_client:
        from rocker.core import get_docker_client

        docker_client = get_docker_client()

    streaming_kwargs = dict(kwargs)
    streaming_kwargs["decode"] = True

    image_id: Optional[str] = None

    for line in docker_client.build(**streaming_kwargs):
        if not isinstance(line, dict):
            continue

        error = line.get("error") or line.get("errorDetail", {}).get("message")
        if error:
            _format_output(f"ERROR: {error}", output_callback)
            return None

        stream = line.get("stream")
        status = line.get("status")
        progress = line.get("progress")

        message: Optional[str] = None
        if stream:
            message = stream.rstrip()
        elif status:
            message = f"{status} {progress}".rstrip() if progress else status

        if message:
            _format_output(message, output_callback)

        if stream:
            match = re.search(r"Successfully built ([0-9a-f]{12,})", stream)
            if match:
                image_id = match.group(1)[:12]

        aux = line.get("aux")
        if not image_id and isinstance(aux, dict):
            aux_id = aux.get("ID")
            if aux_id:
                if aux_id.startswith("sha256:"):
                    aux_id = aux_id.split(":", 1)[1]
                image_id = aux_id[:12]

    if image_id:
        return image_id

    _format_output("no more output and success not detected", output_callback)
    return None


def _patch_rocker_docker_build() -> None:
    # Defer import so rocker remains optional at installation time
    try:
        import rocker.core as core
    except Exception:
        return

    if getattr(core, "_deps_rocker_buildkit_patch", False):
        return

    def patched_docker_build(
        docker_client=None,
        output_callback: Optional[Callable[[str], None]] = None,
        **kwargs: Any,
    ) -> Optional[str]:
        if _buildkit_requested():
            path = kwargs.get("path")
            if path:
                buildargs_for_cli = kwargs.get("buildargs")
                if isinstance(buildargs_for_cli, str):
                    try:
                        import json

                        buildargs_for_cli = json.loads(buildargs_for_cli)
                    except Exception:
                        buildargs_for_cli = None
                dockerfile = kwargs.get("dockerfile")
                try:
                    image_id = _docker_build_with_cli(
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

        return _docker_build_with_sdk(
            docker_client,
            output_callback,
            **kwargs,
        )

    core.docker_build = patched_docker_build
    core._deps_rocker_buildkit_patch = True


_patch_rocker_docker_build()
