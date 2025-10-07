"""Utilities for reasoning about Docker BuildKit configuration."""

from __future__ import annotations

from typing import Mapping, MutableMapping, Optional
import os


def is_buildkit_enabled(env: Optional[Mapping[str, str]] = None) -> bool:
    """Return True when BuildKit is explicitly enabled in the provided environment."""

    source: Mapping[str, str]
    if env is not None:
        source = env
    else:
        # os.environ has the mapping interface we need but is mutable
        source = os.environ

    value = source.get("DOCKER_BUILDKIT", "")
    return value.strip().lower() in {"1", "true", "yes"}


def ensure_buildkit(env: Optional[MutableMapping[str, str]] = None) -> Mapping[str, str]:
    """Return an env mapping that has DOCKER_BUILDKIT forced to an enabled state."""

    target: MutableMapping[str, str]
    if env is None:
        target = dict(os.environ)
    else:
        target = env

    target.setdefault("DOCKER_BUILDKIT", "1")
    return target
