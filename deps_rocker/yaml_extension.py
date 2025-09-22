import yaml
import pkgutil
from typing import Dict, Any, Optional
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class YamlRockerExtension(SimpleRockerExtension):
    """Extension that loads configuration from YAML files"""

    def __init__(self, yaml_config: Optional[Dict[str, Any]] = None):
        # Initialize parent class first
        super().__init__()

        if yaml_config:
            self._load_from_dict(yaml_config)
        else:
            self._load_from_yaml()

    def _load_from_yaml(self):
        """Load configuration from YAML file in the extension package"""
        try:
            pkg = self._get_pkg()
            yaml_file = f"{self.name}.yaml"
            dat = pkgutil.get_data(pkg, yaml_file)
            if dat is not None:
                config = yaml.safe_load(dat.decode("utf-8"))
                self._load_from_dict(config)
        except Exception as e:
            # Fallback to default behavior if YAML loading fails
            pass

    def _load_from_dict(self, config: Dict[str, Any]):
        """Load configuration from dictionary"""
        # Set basic properties
        if "name" in config:
            self.name = config["name"]
        if "description" in config:
            self.__doc__ = config["description"]
        if "apt_packages" in config:
            self.apt_packages = config["apt_packages"]
        if "depends_on_extension" in config:
            self.depends_on_extension = tuple(config["depends_on_extension"])
        if "empy_args" in config:
            self.empy_args = config["empy_args"]
        if "empy_user_args" in config:
            self.empy_user_args = config["empy_user_args"]


def create_extension_from_yaml(yaml_path: str) -> type:
    """Create an extension class from a YAML configuration file"""
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    class_name = config.get("class_name", config["name"].replace("_", "").title())

    # Create dynamic class
    class DynamicExtension(YamlRockerExtension):
        def __init__(self):
            super().__init__(config)

    DynamicExtension.__name__ = class_name
    DynamicExtension.__qualname__ = class_name

    return DynamicExtension