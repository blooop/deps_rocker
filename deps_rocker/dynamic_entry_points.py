"""
Dynamic entry point generator for YAML-based extensions.
This module provides classes that are loaded from YAML files for use as entry points.
"""

from deps_rocker.yaml_loader import YamlExtensionLoader
from pathlib import Path

# Load classes at module level so they can be used as direct entry points
Curl = YamlExtensionLoader.load_extension_from_file(
    Path(__file__).parent / "extensions" / "curl" / "curl.yaml"
)

NeoVim = YamlExtensionLoader.load_extension_from_file(
    Path(__file__).parent / "extensions" / "neovim" / "neovim.yaml"
)

GitClone = YamlExtensionLoader.load_extension_from_file(
    Path(__file__).parent / "extensions" / "git_clone" / "git_clone.yaml"
)

UV = YamlExtensionLoader.load_extension_from_file(
    Path(__file__).parent / "extensions" / "uv" / "uv.yaml"
)