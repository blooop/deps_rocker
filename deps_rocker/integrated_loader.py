"""
Integrated loader that combines both static extensions and dynamic YAML extensions.
This ensures backward compatibility while enabling the new dynamic system.
"""

from typing import Dict, Type
from deps_rocker.dynamic_yaml_loader import DynamicYamlLoader


class IntegratedExtensionLoader:
    """Loads both static Python extensions and dynamic YAML extensions"""

    @staticmethod
    def get_all_extensions() -> Dict[str, Type]:
        """
        Get all available extensions from both static and dynamic sources.

        Returns:
            Dictionary mapping extension names to extension classes
        """
        extensions = {}

        # Load dynamic YAML extensions
        try:
            yaml_extensions = DynamicYamlLoader.get_all_extensions()
            extensions.update(yaml_extensions)
        except Exception:
            # Fail silently for dynamic extensions
            pass

        return extensions

    @staticmethod
    def get_extension(name: str) -> Type:
        """Get a specific extension by name"""
        extensions = IntegratedExtensionLoader.get_all_extensions()
        if name in extensions:
            return extensions[name]
        raise ImportError(f"Extension '{name}' not found")


# For backward compatibility, expose common functions
def get_all_extensions():
    """Get all extensions"""
    return IntegratedExtensionLoader.get_all_extensions()


def get_extension(name: str):
    """Get specific extension by name"""
    return IntegratedExtensionLoader.get_extension(name)