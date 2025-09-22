"""
Dynamic YAML extension loader for deps_rocker.
Automatically discovers and loads extensions from any xxx.deps_rocker.yaml file.
"""

import os
import yaml
from pathlib import Path
from typing import Dict, Type, List, Set
from deps_rocker.yaml_extension import YamlRockerExtension


class DynamicYamlLoader:
    """Dynamically discovers and loads extensions from .deps_rocker.yaml files"""

    @staticmethod
    def discover_yaml_extensions(search_paths: List[str] = None) -> Dict[str, Type]:
        """
        Discover and load all YAML extensions from .deps_rocker.yaml files.

        Args:
            search_paths: List of paths to search for YAML files.
                         If None, searches current directory and all subdirectories.

        Returns:
            Dictionary mapping extension names to extension classes
        """
        if search_paths is None:
            # Default to current working directory and common locations
            search_paths = [
                os.getcwd(),  # Current directory
                os.path.expanduser("~"),  # Home directory
                "/workspace",  # Common container workspace
                "/tmp",  # Temporary directory
            ]

        extensions = {}
        processed_files: Set[str] = set()

        for search_path in search_paths:
            search_path = Path(search_path)
            if not search_path.exists():
                continue

            # Find all .deps_rocker.yaml files recursively
            yaml_files = list(search_path.rglob("*.deps_rocker.yaml"))

            for yaml_file in yaml_files:
                # Avoid processing the same file multiple times
                file_key = str(yaml_file.resolve())
                if file_key in processed_files:
                    continue
                processed_files.add(file_key)

                try:
                    extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)
                    if extension_class:
                        # Use the extension name from YAML as the key
                        extensions[extension_class.name] = extension_class
                except Exception as e:
                    # Silently continue - don't pollute output with warnings
                    # Users can enable verbose mode if they need debugging
                    continue

        return extensions

    @staticmethod
    def load_extension_from_file(yaml_file: Path) -> Type[YamlRockerExtension]:
        """
        Load a single extension from a .deps_rocker.yaml file.

        Args:
            yaml_file: Path to the YAML configuration file

        Returns:
            Extension class created from YAML configuration
        """
        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        if not config:
            raise ValueError(f"YAML file {yaml_file} is empty or invalid")

        # Extract extension name from filename if not specified in config
        filename = yaml_file.stem  # e.g., "myext.deps_rocker" -> "myext"
        if filename.endswith('.deps_rocker'):
            default_name = filename[:-12]  # Remove ".deps_rocker" suffix
        else:
            default_name = filename

        extension_name = config.get('name', default_name)
        # Properly capitalize class name: convert underscores/hyphens to title case
        if 'class_name' in config:
            class_name = config['class_name']
        else:
            # Split on underscores and hyphens, capitalize each part, then join
            parts = extension_name.replace('-', '_').split('_')
            class_name = ''.join(part.capitalize() for part in parts)

        # Set name in config if not present
        if 'name' not in config:
            config['name'] = extension_name

        # Handle dockerfile configuration
        dockerfile_path = yaml_file.parent / f"{extension_name}.Dockerfile"
        if dockerfile_path.exists():
            # If there's a companion Dockerfile, reference it
            config.setdefault('dockerfile', str(dockerfile_path))

        # Create dynamic class
        class DynamicYamlExtension(YamlRockerExtension):
            def __init__(self, *args, **kwargs):
                # Initialize the base classes properly
                super().__init__(yaml_config=config)

            def _get_dockerfile_path(self):
                """Get path to companion Dockerfile if it exists"""
                dockerfile_path = yaml_file.parent / f"{extension_name}.Dockerfile"
                return dockerfile_path if dockerfile_path.exists() else None

        # Set class metadata
        DynamicYamlExtension.__name__ = class_name
        DynamicYamlExtension.__qualname__ = class_name
        DynamicYamlExtension.name = extension_name
        DynamicYamlExtension.__module__ = f"deps_rocker.dynamic.{extension_name}"

        # Store source file for debugging
        DynamicYamlExtension._yaml_source = str(yaml_file)

        # Store dockerfile path as class attribute
        dockerfile_path = yaml_file.parent / f"{extension_name}.Dockerfile"
        DynamicYamlExtension._dockerfile_path = str(dockerfile_path) if dockerfile_path.exists() else None

        return DynamicYamlExtension

    @staticmethod
    def get_all_extensions(additional_search_paths: List[str] = None) -> Dict[str, Type]:
        """
        Get all discovered YAML extensions.
        This can be used by the entry point system.

        Args:
            additional_search_paths: Additional paths to search beyond defaults

        Returns:
            Dictionary of extension name to extension class
        """
        search_paths = additional_search_paths or []
        return DynamicYamlLoader.discover_yaml_extensions(search_paths)


def create_dynamic_extension_getter(extension_name: str):
    """
    Create a function that returns a specific extension class.
    This is used for entry point compatibility.
    """
    def get_extension():
        extensions = DynamicYamlLoader.get_all_extensions()
        if extension_name in extensions:
            return extensions[extension_name]
        raise ImportError(f"Dynamic YAML extension '{extension_name}' not found")
    return get_extension


# Convenience function for backward compatibility
def get_yaml_extensions() -> Dict[str, Type]:
    """Get all discovered YAML extensions."""
    return DynamicYamlLoader.get_all_extensions()