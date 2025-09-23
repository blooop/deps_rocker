"""
Dynamic entry points for YAML-based extensions.
This module provides dynamic entry point functions that rocker can discover.
"""

from typing import Type
from deps_rocker.dynamic_yaml_loader import DynamicYamlLoader
import os


def get_curl_extension() -> Type:
    """Dynamic entry point for curl extension"""
    extensions_dir = os.path.join(os.path.dirname(__file__), "extensions")
    extensions = DynamicYamlLoader.discover_yaml_extensions([extensions_dir])
    if "curl" in extensions:
        return extensions["curl"]
    raise ImportError("curl extension not found in YAML files")


def get_uv_extension() -> Type:
    """Dynamic entry point for uv extension"""
    extensions_dir = os.path.join(os.path.dirname(__file__), "extensions")
    extensions = DynamicYamlLoader.discover_yaml_extensions([extensions_dir])
    if "uv" in extensions:
        return extensions["uv"]
    raise ImportError("uv extension not found in YAML files")


def get_locales_yaml_extension() -> Type:
    """Dynamic entry point for locales YAML extension"""
    extensions_dir = os.path.join(os.path.dirname(__file__), "extensions")
    extensions = DynamicYamlLoader.discover_yaml_extensions([extensions_dir])
    if "locales" in extensions:
        return extensions["locales"]
    raise ImportError("locales extension not found in YAML files")


def get_pixi_yaml_extension() -> Type:
    """Dynamic entry point for pixi YAML extension"""
    extensions_dir = os.path.join(os.path.dirname(__file__), "extensions")
    extensions = DynamicYamlLoader.discover_yaml_extensions([extensions_dir])
    if "pixi" in extensions:
        return extensions["pixi"]
    raise ImportError("pixi extension not found in YAML files")
