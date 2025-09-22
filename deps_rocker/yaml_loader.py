"""
Automatic YAML extension loader for deps_rocker.
Scans for .yaml files and dynamically creates extension classes.
"""

import os
import yaml
import pkgutil
from pathlib import Path
from typing import Dict, Type, List
from deps_rocker.yaml_extension import YamlRockerExtension


class YamlExtensionLoader:
    """Loads extensions from YAML configuration files"""

    @staticmethod
    def discover_yaml_extensions(search_paths: List[str] = None) -> Dict[str, Type]:
        """
        Discover and load all YAML extensions from specified paths.

        Args:
            search_paths: List of paths to search for YAML files.
                         If None, uses default deps_rocker extensions path.

        Returns:
            Dictionary mapping extension names to extension classes
        """
        if search_paths is None:
            # Default to deps_rocker extensions directory
            import deps_rocker.extensions
            extensions_path = Path(deps_rocker.extensions.__file__).parent
            search_paths = [str(extensions_path)]

        extensions = {}

        for search_path in search_paths:
            search_path = Path(search_path)
            if not search_path.exists():
                continue

            # Recursively find all .yaml files
            yaml_files = list(search_path.rglob("*.yaml"))

            for yaml_file in yaml_files:
                try:
                    extension_class = YamlExtensionLoader.load_extension_from_file(yaml_file)
                    if extension_class:
                        # Use the extension name from YAML as the key
                        extensions[extension_class.name] = extension_class
                except Exception as e:
                    print(f"Warning: Failed to load YAML extension from {yaml_file}: {e}")
                    continue

        return extensions

    @staticmethod
    def load_extension_from_file(yaml_file: Path) -> Type[YamlRockerExtension]:
        """
        Load a single extension from a YAML file.

        Args:
            yaml_file: Path to the YAML configuration file

        Returns:
            Extension class created from YAML configuration
        """
        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        if not config or 'name' not in config:
            raise ValueError(f"YAML file {yaml_file} missing required 'name' field")

        extension_name = config['name']
        class_name = config.get('class_name', extension_name.replace('_', '').title())

        # Create dynamic class
        class DynamicYamlExtension(YamlRockerExtension):
            def __init__(self):
                # Initialize the base classes properly
                super().__init__(yaml_config=config)

        # Set class metadata
        DynamicYamlExtension.__name__ = class_name
        DynamicYamlExtension.__qualname__ = class_name
        DynamicYamlExtension.name = extension_name
        DynamicYamlExtension.__module__ = f"deps_rocker.extensions.{extension_name}"

        # Store source file for debugging
        DynamicYamlExtension._yaml_source = str(yaml_file)

        return DynamicYamlExtension


def get_yaml_extensions() -> Dict[str, Type]:
    """
    Convenience function to get all discovered YAML extensions.
    This can be used by the entry point system.
    """
    return YamlExtensionLoader.discover_yaml_extensions()


# For compatibility with entry point system, create individual extension getters
def create_extension_getter(extension_name: str):
    """Create a function that returns a specific extension class"""
    def get_extension():
        extensions = get_yaml_extensions()
        if extension_name in extensions:
            return extensions[extension_name]
        raise ImportError(f"YAML extension '{extension_name}' not found")
    return get_extension