"""Helpers for running Docker builds via the docker SDK."""

from __future__ import annotations

import re
from typing import Any, Callable, Optional, Tuple

OutputCallback = Optional[Callable[[str], None]]


def _emit(message: Optional[str], output_callback: OutputCallback) -> None:
    if message and output_callback:
        output_callback(message)


def parse_sdk_stream(
    chunk: dict[str, Any], output_callback: OutputCallback
) -> Tuple[Optional[str], bool]:
    """Parse a single chunk from docker client's build stream.

    Returns a tuple of (image_id, is_error).
    """

    if error := chunk.get("error") or chunk.get("errorDetail", {}).get("message"):
        _emit(f"ERROR: {error}", output_callback)
        return None, True

    message = None
    if stream := chunk.get("stream"):
        message = stream.rstrip()
    elif status := chunk.get("status"):
        progress = chunk.get("progress")
        message = f"{status} {progress}".rstrip() if progress else status

    _emit(message, output_callback)

    if stream and (match := re.search(r"Successfully built ([0-9a-f]{12,})", stream)):
        return match.group(1), False

    if isinstance(aux := chunk.get("aux"), dict):
        if aux_id := aux.get("ID"):
            return aux_id.split(":", 1)[1] if aux_id.startswith("sha256:") else aux_id, False

    return None, False


def docker_build_with_sdk(
    docker_client,
    output_callback: OutputCallback,
    **kwargs: Any,
) -> Optional[str]:
    if not docker_client:
        from rocker.core import get_docker_client

        docker_client = get_docker_client()

    streaming_kwargs = dict(kwargs)
    streaming_kwargs["decode"] = True

    for chunk in docker_client.build(**streaming_kwargs):
        if not isinstance(chunk, dict):
            continue
        image_id, is_error = parse_sdk_stream(chunk, output_callback)
        if is_error:
            return None
        if image_id:
            return image_id

    _emit("no more output and success not detected", output_callback)
    return None
