"""
Dynamic extension registry that provides individual entry points for each YAML extension.
This module creates entry point functions that rocker can discover.
"""

import os
from typing import Type
from deps_rocker.dynamic_yaml_loader import DynamicYamlLoader


def _get_extensions_dir():
    """Get the path to the extensions directory"""
    return os.path.join(os.path.dirname(__file__), "extensions")


def _get_yaml_extension(name: str) -> Type:
    """Get a specific YAML extension by name"""
    extensions_dir = _get_extensions_dir()
    extensions = DynamicYamlLoader.discover_yaml_extensions([extensions_dir])
    if name in extensions:
        return extensions[name]
    raise ImportError(f"YAML extension '{name}' not found")


# Cache for loaded extensions
_extension_cache = {}

def _get_cached_extension(name: str) -> Type:
    """Get a cached extension or load it if not cached"""
    if name not in _extension_cache:
        _extension_cache[name] = _get_yaml_extension(name)
    return _extension_cache[name]

# Individual extension classes that are created dynamically
Curl = _get_cached_extension('curl')
UV = _get_cached_extension('uv')
LocalesYaml = _get_cached_extension('locales')
PixiYaml = _get_cached_extension('pixi')


# For debugging - function to list all available YAML extensions
def list_yaml_extensions():
    """List all discovered YAML extensions"""
    extensions_dir = _get_extensions_dir()
    return list(DynamicYamlLoader.discover_yaml_extensions([extensions_dir]).keys())