# deps_rocker

This is a [rocker](https://github.com/tfoote/rocker) extension for automating dependency installation.  The aim is to allow a projects to define its development dependencies in a deps.yaml file which are added to the rocker container. The extension will recursivly search for deps.yaml files and run the install commands in several layers.  

Layer order:
- env

- scripts_tools
- apt_tools
- pip_tools

- scripts_base
- apt_base
- pip_base

- scripts
- apt
- pip

- pyproject_toml #reads from all pyproject.toml files

- scripts_post

If rocker is used to launch from a folder that contains multple projects with deps.yaml it will create a container to enable development of all of them combined together. 


example deps.yaml

```

env:
  - BASIC_ENV_VAR=1

apt_tools: #install basic development tools which almost never change
  - git
  - git-lfs
  - python3-pip

pip_tools: #install basic development tools which almost never change
  - pip #this updates pip to latest version
  - flit
  - pytest
  - ruff

pip: #project pip dependencies that may change on a more regular basis
  - pyyaml

```

run with:

```
rocker --deps-dependencies ubuntu:22.04
```



## limitations/TODO

This has only been tested on the ubuntu base image. It assumes you have access to apt-get.
