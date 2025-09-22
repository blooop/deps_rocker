import yaml
import pkgutil
from typing import Dict, Any, Optional, List
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class YamlRockerExtension(SimpleRockerExtension):
    """Extension that loads configuration from YAML files"""

    def __init__(self, yaml_config: Optional[Dict[str, Any]] = None, docker_compose_config: Optional[Dict[str, Any]] = None):
        # Initialize parent class first
        super().__init__()

        # Store config for later use
        self._yaml_config = yaml_config or {}
        self._docker_compose_config = docker_compose_config or {}
        self._dockerfile_path = None
        self._docker_compose_commands = []

        if yaml_config:
            self._load_from_dict(yaml_config)
        else:
            self._load_from_yaml()

        if docker_compose_config:
            self._load_from_docker_compose(docker_compose_config)

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
        if "dockerfile" in config:
            self._dockerfile_path = config["dockerfile"]

    def _load_from_docker_compose(self, docker_compose_config: Dict[str, Any]):
        """Load configuration from Docker Compose and convert to Dockerfile commands"""
        services = docker_compose_config.get('services', {})

        for service_config in services.values():
            # Convert environment variables
            env_vars = service_config.get('environment', {})
            if isinstance(env_vars, list):
                # Handle list format: ["VAR=value", "VAR2=value2"]
                for env_var in env_vars:
                    if '=' in env_var:
                        self._docker_compose_commands.append(f"ENV {env_var}")
            elif isinstance(env_vars, dict):
                # Handle dict format: {"VAR": "value", "VAR2": "value2"}
                for key, value in env_vars.items():
                    self._docker_compose_commands.append(f"ENV {key} {value}")

            # Convert volumes (working directory setup)
            volumes = service_config.get('volumes', [])
            for volume in volumes:
                if isinstance(volume, str) and ':' in volume:
                    # Format: "./local/path:/container/path"
                    local_path, container_path = volume.split(':', 1)
                    if not local_path.startswith('.'):
                        # Skip bind mounts for now, focus on working directories
                        continue
                    self._docker_compose_commands.append(f"WORKDIR {container_path}")
                    break  # Only set one working directory

            # Convert working directory
            working_dir = service_config.get('working_dir')
            if working_dir:
                self._docker_compose_commands.append(f"WORKDIR {working_dir}")

            # Convert ports (expose them)
            ports = service_config.get('ports', [])
            for port in ports:
                if isinstance(port, str):
                    # Format: "8080:8080" or "8080"
                    if ':' in port:
                        _, container_port = port.split(':', 1)
                        self._docker_compose_commands.append(f"EXPOSE {container_port}")
                    else:
                        self._docker_compose_commands.append(f"EXPOSE {port}")
                elif isinstance(port, int):
                    self._docker_compose_commands.append(f"EXPOSE {port}")

            # Convert commands
            command = service_config.get('command')
            if command:
                if isinstance(command, list):
                    cmd_str = ' '.join(command)
                else:
                    cmd_str = command
                # Store as a comment for now, since RUN executes at build time
                self._docker_compose_commands.append(f"# Service command: {cmd_str}")

    def get_dockerfile_path(self):
        """Get path to companion Dockerfile if it exists"""
        return getattr(self, '_dockerfile_path', None)

    def get_snippet(self, cliargs) -> str:
        """Override to include custom Dockerfile and Docker Compose commands"""
        # Get the original snippet from parent class
        snippet = super().get_snippet(cliargs)

        # Add custom Dockerfile content if specified
        if self._dockerfile_path:
            try:
                from pathlib import Path
                dockerfile_path = Path(self._dockerfile_path)
                if not dockerfile_path.is_absolute():
                    # Relative path - resolve relative to the extension package
                    pkg = self._get_pkg()
                    import pkgutil
                    dat = pkgutil.get_data(pkg, self._dockerfile_path)
                    if dat is not None:
                        dockerfile_content = dat.decode("utf-8").strip()
                        if snippet:
                            snippet = f"{snippet}\n\n{dockerfile_content}"
                        else:
                            snippet = dockerfile_content
                elif dockerfile_path.exists():
                    # Absolute path
                    with open(dockerfile_path, 'r', encoding='utf-8') as f:
                        dockerfile_content = f.read().strip()
                        if snippet:
                            snippet = f"{snippet}\n\n{dockerfile_content}"
                        else:
                            snippet = dockerfile_content
            except Exception as e:
                # If we can't read the Dockerfile, continue without it
                pass

        # Add Docker Compose commands if any
        if self._docker_compose_commands:
            compose_snippet = "\n".join(self._docker_compose_commands)
            if snippet:
                snippet = f"{snippet}\n\n# Docker Compose configuration\n{compose_snippet}"
            else:
                snippet = f"# Docker Compose configuration\n{compose_snippet}"

        return snippet


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