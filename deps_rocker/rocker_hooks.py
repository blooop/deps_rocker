"""
Hooks to integrate YAML-based extensions into rocker's discovery mechanism.
This module patches rocker's plugin discovery to include YAML extensions automatically.
"""

import os
from typing import Dict, Type
from deps_rocker.dynamic_yaml_loader import DynamicYamlLoader


def patch_rocker_extension_discovery():
    """
    Patch rocker's extension discovery to include YAML-based extensions.
    This function modifies rocker's list_plugins function to automatically
    include dynamically discovered YAML extensions.
    """
    try:
        import rocker.core

        # Store the original function
        if not hasattr(rocker.core, '_original_list_plugins'):
            rocker.core._original_list_plugins = rocker.core.list_plugins

        def enhanced_list_plugins(extension_point='rocker.extensions'):
            """Enhanced version that includes YAML extensions"""
            # Get the original static extensions
            plugins = rocker.core._original_list_plugins(extension_point)

            # Add YAML extensions if this is the rocker extensions group
            if extension_point == 'rocker.extensions':
                try:
                    # Search for YAML extensions in the deps_rocker extensions directory
                    extensions_dir = os.path.join(os.path.dirname(__file__), "extensions")
                    yaml_extensions = DynamicYamlLoader.discover_yaml_extensions([extensions_dir])

                    # Merge YAML extensions, but don't override existing ones
                    for name, ext_class in yaml_extensions.items():
                        if name not in plugins:
                            plugins[name] = ext_class
                except Exception:
                    # Silently continue if YAML discovery fails
                    pass

            return plugins

        # Replace rocker's list_plugins function
        rocker.core.list_plugins = enhanced_list_plugins

    except ImportError:
        # rocker not available, skip patching
        pass


def get_yaml_extension_registry() -> Dict[str, Type]:
    """
    Get a registry of all discovered YAML extensions.
    This can be used by other parts of the system.
    """
    extensions_dir = os.path.join(os.path.dirname(__file__), "extensions")
    return DynamicYamlLoader.discover_yaml_extensions([extensions_dir])


# Auto-patch when this module is imported
patch_rocker_extension_discovery()