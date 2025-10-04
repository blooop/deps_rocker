"""Helpers for running Docker builds via the docker CLI."""

from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, MutableMapping, Optional, Sequence

from deps_rocker.buildkit import ensure_buildkit

OutputCallback = Optional[Callable[[str], None]]


def _prepare_env(env: Optional[MutableMapping[str, str]] = None) -> MutableMapping[str, str]:
    """Return an env mapping with BuildKit guaranteed to be enabled."""

    if env is None:
        env = dict(os.environ)
    ensure_buildkit(env)
    return env


def _build_cli_command(
    *,
    path: str,
    tag: Optional[str],
    rm: bool,
    nocache: bool,
    pull: bool,
    dockerfile: Optional[str],
    buildargs: Optional[Dict[str, Any]],
) -> list[str]:
    """Construct a docker build command."""

    cmd = ["docker", "build", "--progress=plain"]

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
    return cmd


def _stream_process_output(process: subprocess.Popen[str], output_callback: OutputCallback) -> None:
    """Forward process output to the optional callback."""

    assert process.stdout is not None
    for raw_line in process.stdout:
        line = raw_line.rstrip("\n")
        if line and output_callback:
            output_callback(line)


def run_docker_cli(
    *,
    command: Sequence[str],
    env: Mapping[str, str],
    output_callback: OutputCallback = None,
) -> Optional[str]:
    """Execute the docker CLI command and return the resulting image identifier if available."""

    with tempfile.TemporaryDirectory() as tmp_dir:
        iid_path = Path(tmp_dir) / "iid"
        full_command = list(command) + [f"--iidfile={iid_path}"]

        # Passing a list keeps the call shell-free, preventing shell injection.
        with subprocess.Popen(
            full_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=dict(env),
        ) as process:
            _stream_process_output(process, output_callback)
            returncode = process.wait()

        if returncode != 0:
            return None

        try:
            image_identifier = iid_path.read_text(encoding="utf-8").strip()
        except FileNotFoundError:
            return None

    if not image_identifier:
        return None

    if image_identifier.startswith("sha256:"):
        image_identifier = image_identifier.split(":", 1)[1]

    return image_identifier


def docker_build_with_cli(
    *,
    path: str,
    tag: Optional[str],
    rm: bool,
    nocache: bool,
    pull: bool,
    dockerfile: Optional[str],
    buildargs: Optional[Dict[str, Any]],
    output_callback: OutputCallback,
    env: Optional[MutableMapping[str, str]] = None,
) -> Optional[str]:
    """High level helper used by the rocker patch to build with the docker CLI."""

    safe_env = _prepare_env(env)
    # Normalise build arguments to string values as expected by the CLI
    normalised_buildargs: Optional[Dict[str, str]] = None
    if buildargs:
        normalised_buildargs = {key: str(value) for key, value in buildargs.items()}

    command = _build_cli_command(
        path=path,
        tag=tag,
        rm=rm,
        nocache=nocache,
        pull=pull,
        dockerfile=dockerfile,
        buildargs=normalised_buildargs,
    )
    return run_docker_cli(command=command, env=safe_env, output_callback=output_callback)
