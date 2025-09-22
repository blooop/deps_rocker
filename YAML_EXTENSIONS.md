# Dynamic YAML Extensions

This system allows you to create deps_rocker extensions using simple YAML files without needing to manually register them in pyproject.toml.

## Creating a YAML Extension

### 1. Create a `.deps_rocker.yaml` file

Create a file named `[extension_name].deps_rocker.yaml` anywhere in your project:

```yaml
name: my_extension
class_name: MyExtension  # Optional, defaults to capitalized name
description: "Description of what this extension does"
apt_packages:
  - htop
  - curl
  - git
depends_on_extension:
  - curl
  - locales
```

### 2. Optional: Create a companion Dockerfile

Create a file named `[extension_name].Dockerfile` in the same directory as your YAML file:

```dockerfile
RUN echo "Installing my custom tools..." && \
    apt-get update && \
    apt-get install -y my-custom-package && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
```

## YAML Configuration Options

- **name**: The extension name (required)
- **class_name**: Python class name (optional, auto-generated if not provided)
- **description**: Description shown in help text
- **apt_packages**: List of apt packages to install
- **depends_on_extension**: List of other extensions this depends on
- **empy_args**: Template arguments for Dockerfile generation
- **empy_user_args**: User-specific template arguments

## How It Works

1. The system automatically discovers all `*.deps_rocker.yaml` files in:
   - Current working directory (recursively)
   - Home directory (recursively)
   - `/workspace` directory (for containers)
   - Any additional paths you specify

2. Each YAML file is dynamically converted into a rocker extension class

3. Extensions are automatically available to rocker without manual registration

## Examples

### Simple Package Installation
```yaml
# tools.deps_rocker.yaml
name: dev_tools
description: "Essential development tools"
apt_packages:
  - vim
  - tmux
  - htop
  - tree
```

### Extension with Dependencies
```yaml
# nodejs.deps_rocker.yaml
name: nodejs
description: "Node.js development environment"
depends_on_extension:
  - curl
apt_packages:
  - nodejs
  - npm
```

### Extension with Custom Dockerfile
```yaml
# python_ml.deps_rocker.yaml
name: python_ml
description: "Python machine learning environment"
depends_on_extension:
  - curl
```

With companion `python_ml.Dockerfile`:
```dockerfile
RUN pip install numpy pandas scikit-learn matplotlib jupyter
```

## Migration from Old System

If you have existing extensions in the `deps_rocker/extensions/` directory, they will continue to work. The new system is additive and doesn't break existing functionality.

To migrate an existing extension:
1. Create a `[name].deps_rocker.yaml` file with the configuration
2. Optionally create a `[name].Dockerfile` for custom installation steps
3. The extension will be automatically discovered and available

## Benefits

- **No registration required**: Just create a YAML file and it's automatically available
- **Portable**: Extensions can be shared by simply copying the YAML (and optional Dockerfile) files
- **Flexible**: Works from any directory, making it perfect for project-specific extensions
- **Backward compatible**: Existing Python-based extensions continue to work unchanged