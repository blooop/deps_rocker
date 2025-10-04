"""deps_rocker package initialisation and rocker integration helpers."""

from __future__ import annotations

from deps_rocker.rocker_patch import patch_rocker_docker_build

__all__ = ["patch_rocker_docker_build"]

patch_rocker_docker_build()
